#!/usr/bin/env python3
"""Config-driven, read-only hardware probe for Vanilla Jazzberry.

Reads a YAML config that carries a ``probe.checks`` list, runs each read-only
check against the Raspberry Pi host, and emits a single JSON report.

Run this ON THE PI (Raspberry Pi OS / Debian). It is read-only: it lists and
queries devices and never configures, writes, or moves anything. Motion and
actuator behavior are deliberately out of scope for this tool.

Some checks are "active": they open a device (the Yahboom board over serial,
the RPLIDAR over serial) and read live telemetry to confirm the device actually
responds, not merely that it enumerates. These still NEVER actuate the vehicle:
they only request read-only telemetry/health. The motor board check reads
firmware/IMU/encoder/battery telemetry but issues no motion command; the LIDAR
check reads INFO/HEALTH and keeps the scan motor disabled. Every active check
degrades to ``unavailable`` (never a crash) when the hardware is off or silent.

Usage:
    python3 probe_hw.py robot_hardware.yaml
    python3 probe_hw.py robot_hardware.yaml --output hardware_probe.json
    python3 probe_hw.py safety_czar.yaml

The same script accepts either YAML file; each file declares the checks that
matter to its concern. Run it once per file.

Dependencies: PyYAML (`pip install pyyaml`). Everything else is stdlib. Optional
host tools (i2c-tools, libcamera, Rosmaster_Lib, pyserial) are used if present
and are reported as ``unavailable`` when missing, never fatal.
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import re
import shutil
import socket
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable

try:
    import yaml
except ImportError:  # pragma: no cover - environment dependent
    sys.stderr.write(
        "ERROR: PyYAML is required. Install it with: python3 -m pip install pyyaml\n"
    )
    raise SystemExit(2)

SCRIPT_VERSION = "0.1.0"

# Status vocabulary. Hard-fail statuses count against deployment readiness when
# the check is marked required.
PASS = "pass"
MISSING = "missing"
UNEXPECTED = "unexpected"
UNAVAILABLE = "unavailable"
PENDING = "pending"
NOT_TESTED = "not_tested"
ERROR = "error"
WARNING = "warning"

HARD_FAIL = {MISSING, UNEXPECTED, UNAVAILABLE, ERROR}

# Known USB VID:PID hints (lowercase, no colon) -> friendly name.
USB_HINTS = {
    "10c4:ea60": "CP2102 UART (Slamtec RPLIDAR A1)",
    "1a86:7523": "CH340 UART (Yahboom / STM32 boards)",
    "1a86:55d4": "CH9102 UART (Yahboom / STM32 boards)",
    "0403:6001": "FTDI FT232 UART",
    "0483:5740": "STMicroelectronics Virtual COM (STM32 CDC)",
}


# --------------------------------------------------------------------------- #
# Low-level helpers (all read-only, never raise)
# --------------------------------------------------------------------------- #
def run(cmd: list[str], timeout: int = 5) -> tuple[int, str, str]:
    """Run a command; never raise. Returns (returncode, stdout, stderr)."""
    if shutil.which(cmd[0]) is None:
        return (127, "", f"{cmd[0]}: not installed")
    try:
        proc = subprocess.run(
            cmd, capture_output=True, text=True, timeout=timeout, check=False
        )
        return (proc.returncode, proc.stdout.strip(), proc.stderr.strip())
    except Exception as exc:  # timeout / permission / anything
        return (1, "", f"{cmd[0]}: {exc}")


def read_text(path: str) -> str:
    try:
        with open(path, "rb") as handle:
            return handle.read().decode(errors="ignore").strip("\x00").strip()
    except Exception:
        return ""


def host_info() -> dict[str, str]:
    return {
        "model": read_text("/proc/device-tree/model"),
        "kernel": run(["uname", "-a"])[1],
        "architecture": run(["uname", "-m"])[1],
    }


def i2c_buses() -> list[int]:
    """Available I2C bus numbers from /dev/i2c-*."""
    buses = []
    for node in glob.glob("/dev/i2c-*"):
        match = re.search(r"i2c-(\d+)$", node)
        if match:
            buses.append(int(match.group(1)))
    return sorted(buses)


def candidate_buses(bus_param: Any) -> list[int]:
    if bus_param in (None, "auto", "any"):
        found = i2c_buses()
        # Prefer bus 1 (Pi user I2C on GPIO2/3), then others.
        return sorted(found, key=lambda b: (b != 1, b))
    try:
        return [int(bus_param)]
    except (TypeError, ValueError):
        return []


def i2cdetect_addresses(bus: int) -> tuple[list[str], str | None]:
    """Return (addresses, error) for a bus using i2cdetect."""
    if shutil.which("i2cdetect") is None:
        return ([], "i2c-tools not installed")
    rc, text, err = run(["i2cdetect", "-y", str(bus)])
    if rc != 0:
        return ([], err or "i2cdetect failed (is I2C enabled?)")
    detected = []
    for row in text.splitlines()[1:]:
        for token in row.split()[1:]:
            if re.fullmatch(r"[0-7][0-9a-f]", token):
                detected.append("0x" + token)
    return (sorted(set(detected)), None)


def i2cget(bus: int, address: str, register: str) -> tuple[str | None, str | None]:
    """Non-destructive single-register read. Returns (value, error)."""
    if shutil.which("i2cget") is None:
        return (None, "i2c-tools not installed")
    rc, out, err = run(["i2cget", "-y", str(bus), address, register])
    if rc != 0:
        return (None, err or "i2cget failed")
    return (out.strip().lower(), None)


def as_hex_int(value: str) -> int | None:
    try:
        return int(value, 16)
    except (TypeError, ValueError):
        return None


# --------------------------------------------------------------------------- #
# Result construction
# --------------------------------------------------------------------------- #
def make_result(
    check: dict,
    status: str,
    observed: Any = None,
    detail: str = "",
    expected: Any = None,
) -> dict:
    result = {
        "id": check.get("id", "unnamed"),
        "type": check.get("type", "unknown"),
        "required": bool(check.get("required", False)),
        "severity": check.get("severity", "info"),
        "expected": expected if expected is not None else check.get("expected"),
        "observed": observed,
        "status": status,
        "detail": detail,
    }
    if check.get("note"):
        result["note"] = check["note"]
    return result


# --------------------------------------------------------------------------- #
# Check implementations
# --------------------------------------------------------------------------- #
def check_host_identity(check: dict, ctx: dict) -> dict:
    expected = str(check.get("expected", "")).strip().lower()
    model = ctx["host"]["model"]
    if not model:
        return make_result(check, ERROR, None, "could not read /proc/device-tree/model")
    normalized = model.lower().replace(" ", "_")
    if expected and expected in normalized:
        return make_result(check, PASS, model)
    return make_result(
        check, UNEXPECTED, model, f"expected token '{expected}' not found", expected
    )


def check_usb_device(check: dict, ctx: dict) -> dict:
    wanted = [str(x).lower() for x in check.get("usb_ids", [])]
    rc, out, err = run(["lsusb"])
    if rc != 0:
        return make_result(check, UNAVAILABLE, None, err or "lsusb unavailable", wanted)
    present = []
    for line in out.splitlines():
        match = re.search(r"ID (\w{4}:\w{4})", line)
        if match:
            present.append(match.group(1).lower())
    hits = [i for i in wanted if i in present]
    if hits:
        friendly = ", ".join(USB_HINTS.get(i, i) for i in hits)
        return make_result(check, PASS, hits, friendly, wanted)
    return make_result(check, MISSING, present, "expected USB id not present", wanted)


def check_serial_port(check: dict, ctx: dict) -> dict:
    wanted = [str(x).lower() for x in check.get("match_usb_ids", [])]
    ports = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
    matches = []
    for port in ports:
        entry: dict[str, Any] = {"device": port, "readable": os.access(port, os.R_OK)}
        rc, out, _ = run(["udevadm", "info", "--query=property", f"--name={port}"])
        if rc == 0:
            vid = re.search(r"ID_VENDOR_ID=(\w+)", out)
            pid = re.search(r"ID_MODEL_ID=(\w+)", out)
            if vid and pid:
                entry["usb_id"] = f"{vid.group(1).lower()}:{pid.group(1).lower()}"
        if not wanted or entry.get("usb_id") in wanted:
            matches.append(entry)
    if not ports:
        return make_result(check, MISSING, [], "no /dev/ttyUSB* or /dev/ttyACM*", wanted)
    if not matches:
        return make_result(check, MISSING, ports, "no port matched expected USB id", wanted)
    unreadable = [m["device"] for m in matches if not m["readable"]]
    detail = ""
    if unreadable:
        detail = f"not readable (add user to 'dialout'): {', '.join(unreadable)}"
    status = WARNING if unreadable else PASS
    return make_result(check, status, matches, detail, wanted)


def check_i2c_device(check: dict, ctx: dict) -> dict:
    address = str(check.get("address", "")).lower()
    for bus in candidate_buses(check.get("bus")):
        addrs, err = i2cdetect_addresses(bus)
        if err:
            return make_result(check, UNAVAILABLE, None, err, address)
        if address in addrs:
            return make_result(check, PASS, {"bus": bus, "address": address}, "", address)
    return make_result(check, MISSING, None, "address not found on any bus", address)


def check_i2c_devid(check: dict, ctx: dict) -> dict:
    address = str(check.get("address", "")).lower()
    register = str(check.get("register", "0x00")).lower()
    expected = str(check.get("expected_value", "")).lower()
    exp_int = as_hex_int(expected)
    for bus in candidate_buses(check.get("bus")):
        addrs, err = i2cdetect_addresses(bus)
        if err:
            return make_result(check, UNAVAILABLE, None, err, expected)
        if address not in addrs:
            continue
        value, verr = i2cget(bus, address, register)
        if verr:
            return make_result(check, UNAVAILABLE, None, verr, expected)
        obs = {"bus": bus, "address": address, "register": register, "value": value}
        if as_hex_int(value) == exp_int:
            return make_result(check, PASS, obs, "device id confirmed", expected)
        return make_result(check, UNEXPECTED, obs, "device id mismatch", expected)
    return make_result(check, MISSING, None, "address not found on any bus", expected)


def check_camera(check: dict, ctx: dict) -> dict:
    for tool in ("rpicam-hello", "libcamera-hello"):
        if shutil.which(tool):
            rc, out, err = run([tool, "--list-cameras"], timeout=8)
            if rc == 0 and out:
                return make_result(check, PASS, out.splitlines(), tool)
            return make_result(check, MISSING, out or err, f"{tool}: no cameras listed")
    rc, out, _ = run(["vcgencmd", "get_camera"])
    if rc == 0 and out:
        status = PASS if "detected=1" in out else MISSING
        return make_result(check, status, out, "vcgencmd (legacy stack)")
    video = sorted(glob.glob("/dev/video*"))
    if video:
        return make_result(check, WARNING, video, "/dev/video* present; model unverified")
    return make_result(check, UNAVAILABLE, None, "no libcamera/vcgencmd and no /dev/video*")


def check_pisugar(check: dict, ctx: dict) -> dict:
    expect_telemetry = bool(check.get("expect_telemetry", False))
    uds = "/tmp/pisugar-server.sock"

    def ask(sendfn) -> str | None:
        try:
            return sendfn("get model")
        except Exception:
            return None

    reachable = None
    if os.path.exists(uds):
        def send_uds(cmd: str) -> str:
            sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            sock.settimeout(2)
            sock.connect(uds)
            sock.sendall((cmd + "\n").encode())
            resp = sock.recv(256).decode().strip()
            sock.close()
            return resp
        reachable = ask(send_uds)
    if reachable is None:
        try:
            def send_tcp(cmd: str) -> str:
                sock = socket.create_connection(("127.0.0.1", 8423), timeout=2)
                sock.sendall((cmd + "\n").encode())
                resp = sock.recv(256).decode().strip()
                sock.close()
                return resp
            reachable = ask(send_tcp)
        except Exception:
            reachable = None

    if reachable:
        return make_result(check, PASS, {"server": reachable}, "pisugar-server reachable")

    # No server. For the "dumb" S Plus this is expected; presence may still show
    # on I2C. Do not hard-fail when telemetry is not expected.
    addrs: list[str] = []
    for bus in candidate_buses("auto"):
        found, _ = i2cdetect_addresses(bus)
        addrs.extend(found)
    if expect_telemetry:
        return make_result(check, MISSING, {"i2c": addrs}, "pisugar-server not reachable")
    return make_result(
        check, PENDING, {"i2c": addrs},
        "no telemetry expected (PiSugar S Plus is hardware-only)",
    )


def check_command_available(check: dict, ctx: dict) -> dict:
    command = str(check.get("command", ""))
    path = shutil.which(command)
    if path:
        return make_result(check, PASS, path, "", command)
    return make_result(check, UNAVAILABLE, None, f"{command} not installed", command)


def check_python_module(check: dict, ctx: dict) -> dict:
    module = str(check.get("module", ""))
    try:
        __import__(module)
        return make_result(check, PASS, module, "importable", module)
    except Exception as exc:
        return make_result(check, UNAVAILABLE, None, str(exc), module)


def check_file_present(check: dict, ctx: dict) -> dict:
    raw = str(check.get("path", ""))
    path = Path(raw)
    if not path.is_absolute():
        path = ctx["config_dir"] / path
    if path.exists():
        return make_result(check, PASS, str(path), "", raw)
    return make_result(check, MISSING, str(path), "file not found", raw)


def check_disk_space(check: dict, ctx: dict) -> dict:
    path = str(check.get("path", "/"))
    min_free_gb = check.get("min_free_gb")
    try:
        usage = shutil.disk_usage(path)
    except Exception as exc:
        return make_result(check, UNAVAILABLE, None, f"disk_usage failed: {exc}", path)
    free_gb = usage.free / 1e9
    observed = {
        "path": path,
        "free_gb": round(free_gb, 2),
        "total_gb": round(usage.total / 1e9, 2),
        "percent_free": round(100.0 * usage.free / usage.total, 1) if usage.total else None,
    }
    if min_free_gb is not None and free_gb < float(min_free_gb):
        return make_result(
            check, WARNING, observed,
            f"only {free_gb:.1f} GB free (< {float(min_free_gb):.1f} GB floor)", path,
        )
    return make_result(check, PASS, observed, "", path)


def check_host_throttling(check: dict, ctx: dict) -> dict:
    """Read Pi power/thermal throttling state via vcgencmd (read-only)."""
    rc, out, err = run(["vcgencmd", "get_throttled"])
    if rc != 0 or "throttled=" not in out:
        return make_result(check, UNAVAILABLE, None, err or "vcgencmd unavailable")
    raw = out.split("throttled=", 1)[1].strip()
    try:
        bits = int(raw, 16)
    except ValueError:
        return make_result(check, UNAVAILABLE, None, f"unparseable value '{raw}'")
    now_flags = {
        "under_voltage": bool(bits & 0x1),
        "arm_freq_capped": bool(bits & 0x2),
        "throttled": bool(bits & 0x4),
        "soft_temp_limit": bool(bits & 0x8),
    }
    since_boot_flags = {
        "under_voltage": bool(bits & 0x10000),
        "arm_freq_capped": bool(bits & 0x20000),
        "throttled": bool(bits & 0x40000),
        "soft_temp_limit": bool(bits & 0x80000),
    }
    temp_rc, temp_out, _ = run(["vcgencmd", "measure_temp"])
    temp = temp_out.split("temp=", 1)[1].strip() if temp_rc == 0 and "temp=" in temp_out else None
    observed = {
        "raw": raw,
        "now": now_flags,
        "since_boot": since_boot_flags,
        "soc_temp": temp,
    }
    if any(now_flags.values()):
        active = ", ".join(k for k, v in now_flags.items() if v)
        return make_result(check, UNEXPECTED, observed, f"active power/thermal fault: {active}", "0x0")
    if any(since_boot_flags.values()):
        past = ", ".join(k for k, v in since_boot_flags.items() if v)
        return make_result(check, WARNING, observed, f"occurred since boot: {past}", "0x0")
    return make_result(check, PASS, observed, "no throttling", "0x0")


# --------------------------------------------------------------------------- #
# Active serial helpers (read-only telemetry; never actuate the vehicle)
# --------------------------------------------------------------------------- #
def port_usb_id(port: str) -> str | None:
    rc, out, _ = run(["udevadm", "info", "--query=property", f"--name={port}"])
    if rc != 0:
        return None
    vid = re.search(r"ID_VENDOR_ID=(\w+)", out)
    pid = re.search(r"ID_MODEL_ID=(\w+)", out)
    if vid and pid:
        return f"{vid.group(1).lower()}:{pid.group(1).lower()}"
    return None


def resolve_serial_port(check: dict, default_ids: list[str]) -> str | None:
    """Resolve a serial device for an active check, preferring stable matches."""
    explicit = check.get("port")
    if explicit:
        return explicit if os.path.exists(explicit) else None
    wanted = [str(x).lower() for x in check.get("match_usb_ids", default_ids)]
    for port in sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")):
        if port_usb_id(port) in wanted:
            return port
    # Fall back to the Yahboom-style stable symlink only if it matches.
    if os.path.exists("/dev/myserial") and port_usb_id("/dev/myserial") in wanted:
        return "/dev/myserial"
    return None


def _any_nonzero(seq: Any) -> bool:
    try:
        return any(abs(float(x)) > 1e-9 for x in seq)
    except (TypeError, ValueError):
        return False


def rosmaster_session(ctx: dict, check: dict) -> tuple[Any, dict, str | None]:
    """Lazily open ONE cached, read-only Rosmaster telemetry session.

    Returns (bot, snapshot, error). The snapshot carries a one-shot read of
    firmware/IMU/encoder/battery telemetry plus a ``_responsive`` flag. No
    motion command is ever issued; only telemetry reporting is enabled.
    """
    cache = ctx.setdefault("_rosmaster", {})
    if cache.get("tried"):
        return cache.get("session"), cache.get("snapshot", {}), cache.get("error")
    cache["tried"] = True

    try:
        from Rosmaster_Lib import Rosmaster
    except Exception as exc:
        cache["error"] = f"Rosmaster_Lib not importable: {exc}"
        return None, {}, cache["error"]

    port = resolve_serial_port(check, ["1a86:7523"])
    if not port:
        cache["error"] = "motor-controller serial port not found (1a86:7523)"
        return None, {}, cache["error"]
    cache["port"] = port

    try:
        bot = Rosmaster(com=port, debug=False)
        try:
            bot.ser.timeout = 1  # bound the reader thread so it never blocks forever
        except Exception:
            pass
        bot.create_receive_threading()
        try:
            # Enable read-only telemetry reporting; this does NOT move the robot.
            bot.set_auto_report_state(True, forever=False)
        except Exception:
            pass
        time.sleep(0.8)  # allow a few telemetry frames to arrive
    except Exception as exc:
        cache["error"] = f"could not open board on {port}: {exc}"
        return None, {}, cache["error"]

    snapshot: dict[str, Any] = {}
    readers = [
        ("version", bot.get_version),
        ("battery_v", bot.get_battery_voltage),
        ("accel", bot.get_accelerometer_data),
        ("gyro", bot.get_gyroscope_data),
        ("mag", bot.get_magnetometer_data),
        ("encoders", bot.get_motor_encoder),
        ("motion", bot.get_motion_data),
    ]
    for name, fn in readers:
        try:
            snapshot[name] = fn()
        except Exception as exc:
            snapshot[name] = None
            snapshot.setdefault("errors", {})[name] = str(exc)

    version = snapshot.get("version")
    battery = snapshot.get("battery_v")
    snapshot["_responsive"] = bool(
        (isinstance(version, (int, float)) and version > 0)
        or (isinstance(battery, (int, float)) and battery > 0.5)
        or _any_nonzero(snapshot.get("accel") or [])
        or _any_nonzero(snapshot.get("gyro") or [])
        or _any_nonzero(snapshot.get("mag") or [])
    )

    cache["session"] = bot
    cache["snapshot"] = snapshot
    return bot, snapshot, None


def check_rosmaster_board(check: dict, ctx: dict) -> dict:
    """Confirm the Yahboom board actually responds (not just that USB enumerates)."""
    bot, snap, err = rosmaster_session(ctx, check)
    if err:
        return make_result(check, UNAVAILABLE, None, err)
    port = ctx.get("_rosmaster", {}).get("port")
    observed = {"port": port, "firmware_version": snap.get("version")}
    if not snap.get("_responsive"):
        return make_result(
            check, UNAVAILABLE, observed,
            "board port opened but no telemetry received (firmware/wiring?)",
        )
    expected = check.get("expected_firmware")
    if expected is not None and str(snap.get("version")) != str(expected):
        observed["expected_firmware"] = expected
        return make_result(
            check, WARNING, observed,
            f"board responsive; firmware {snap.get('version')} != expected {expected}",
        )
    return make_result(check, PASS, observed, "board responded to telemetry")


def check_rosmaster_imu(check: dict, ctx: dict) -> dict:
    """Read the Yahboom onboard 9-axis IMU over serial (navigation sensor)."""
    bot, snap, err = rosmaster_session(ctx, check)
    if err:
        return make_result(check, UNAVAILABLE, None, err)
    if not snap.get("_responsive"):
        return make_result(check, UNAVAILABLE, None, "board not responding over serial")
    observed = {
        "accel": snap.get("accel"),
        "gyro": snap.get("gyro"),
        "mag": snap.get("mag"),
    }
    if _any_nonzero(snap.get("accel") or []):
        return make_result(check, PASS, observed, "onboard 9-axis IMU reporting")
    return make_result(
        check, WARNING, observed,
        "board responds but IMU accel reads all-zero (chip/report issue)",
    )


def check_rosmaster_encoders(check: dict, ctx: dict) -> dict:
    """Confirm the drive-motor encoder channels report over serial."""
    bot, snap, err = rosmaster_session(ctx, check)
    if err:
        return make_result(check, UNAVAILABLE, None, err)
    if not snap.get("_responsive"):
        return make_result(check, UNAVAILABLE, None, "board not responding over serial")
    encoders = snap.get("encoders")
    expected = check.get("expected_count", 4)
    if not isinstance(encoders, (list, tuple)):
        return make_result(check, UNAVAILABLE, encoders, "encoder telemetry unavailable", expected)
    observed = {"encoders": list(encoders), "count": len(encoders)}
    if len(encoders) < int(expected):
        return make_result(
            check, UNEXPECTED, observed,
            f"only {len(encoders)} encoder channels (< {expected})", expected,
        )
    return make_result(
        check, PASS, observed,
        f"{len(encoders)} encoder channels readable (spin wheels to confirm counts change)",
        expected,
    )


def check_rosmaster_battery(check: dict, ctx: dict) -> dict:
    """Read battery voltage from the Yahboom board (read-only)."""
    bot, snap, err = rosmaster_session(ctx, check)
    if err:
        return make_result(check, UNAVAILABLE, None, err)
    if not snap.get("_responsive"):
        return make_result(check, UNAVAILABLE, None, "board not responding over serial")
    voltage = snap.get("battery_v")
    if not isinstance(voltage, (int, float)):
        return make_result(check, UNAVAILABLE, voltage, "battery voltage unavailable")
    observed = {"voltage": round(float(voltage), 2)}
    min_v = check.get("min_voltage")
    max_v = check.get("max_voltage")
    if min_v is not None and float(voltage) < float(min_v):
        return make_result(check, UNEXPECTED, observed, f"voltage {voltage:.2f}V below {min_v}V")
    if max_v is not None and float(voltage) > float(max_v):
        return make_result(check, UNEXPECTED, observed, f"voltage {voltage:.2f}V above {max_v}V")
    return make_result(check, PASS, observed, f"battery {float(voltage):.2f}V")


def _rplidar_query(port: str, request: int, timeout: int = 2) -> tuple[bytes | None, str | None]:
    """Send a single read-only RPLIDAR request; keep the scan motor OFF."""
    try:
        import serial  # pyserial
    except Exception as exc:
        return None, f"pyserial not available: {exc}"
    try:
        ser = serial.Serial(port, 115200, timeout=timeout)
    except Exception as exc:
        return None, f"could not open {port}: {exc}"
    try:
        try:
            ser.dtr = False  # DTR controls the A1 scan motor; keep it stopped
        except Exception:
            pass
        ser.reset_input_buffer()
        ser.write(bytes([0xA5, request & 0xFF]))
        descriptor = ser.read(7)
        if len(descriptor) < 7 or descriptor[0] != 0xA5 or descriptor[1] != 0x5A:
            return None, "no/invalid response descriptor (device silent?)"
        data_len = (
            descriptor[2]
            | (descriptor[3] << 8)
            | (descriptor[4] << 16)
            | ((descriptor[5] & 0x3F) << 24)
        )
        payload = ser.read(data_len)
        if len(payload) < data_len:
            return None, "truncated payload"
        return payload, None
    except Exception as exc:
        return None, f"query failed: {exc}"
    finally:
        try:
            ser.close()
        except Exception:
            pass


def check_rplidar_health(check: dict, ctx: dict) -> dict:
    """Confirm the RPLIDAR responds and report its HEALTH (read-only, no scan)."""
    port = resolve_serial_port(check, ["10c4:ea60"])
    if not port:
        return make_result(check, MISSING, None, "RPLIDAR serial port not found (10c4:ea60)")

    health, herr = _rplidar_query(port, 0x52)  # GET_HEALTH
    if herr:
        return make_result(check, UNAVAILABLE, {"port": port}, f"health query: {herr}")
    status = health[0] if len(health) >= 1 else None
    error_code = (health[1] | (health[2] << 8)) if len(health) >= 3 else None
    status_name = {0: "good", 1: "warning", 2: "error"}.get(status, "unknown")
    observed: dict[str, Any] = {
        "port": port,
        "health": {"status": status_name, "error_code": error_code},
    }

    info, ierr = _rplidar_query(port, 0x50)  # GET_INFO
    if not ierr and info and len(info) >= 20:
        observed["info"] = {
            "model": info[0],
            "firmware": f"{info[2]}.{info[1]}",
            "hardware": info[3],
            "serial": info[4:20].hex(),
        }

    if status == 0:
        return make_result(check, PASS, observed, "RPLIDAR healthy")
    if status == 1:
        return make_result(check, WARNING, observed, f"RPLIDAR warning (code {error_code})")
    if status == 2:
        return make_result(check, UNEXPECTED, observed, f"RPLIDAR error state (code {error_code})")
    return make_result(check, UNAVAILABLE, observed, "unrecognized health status")


def check_gpio_input(check: dict, ctx: dict) -> dict:
    """Read a single GPIO line level (e.g. the crash/bump switch).

    A passive switch cannot be 'detected' when idle, so this only reports the
    pin's current level once it is wired. If no numeric pin is configured the
    check is NOT_TESTED, documenting intent without a false pass/fail.
    """
    pin = check.get("gpio_pin")
    if not isinstance(pin, int):
        return make_result(
            check, NOT_TESTED, {"gpio_pin": pin},
            "GPIO pin not yet wired/configured (passive switch is not auto-detectable)",
        )
    pull = str(check.get("pull", "none")).lower()
    try:
        import lgpio
    except Exception as exc:
        return make_result(check, UNAVAILABLE, {"gpio_pin": pin}, f"lgpio unavailable: {exc}")
    handle = None
    try:
        handle = lgpio.gpiochip_open(0)
        flags = 0
        if pull == "up":
            flags = getattr(lgpio, "SET_BIAS_PULL_UP", 0)
        elif pull == "down":
            flags = getattr(lgpio, "SET_BIAS_PULL_DOWN", 0)
        elif pull == "none":
            flags = getattr(lgpio, "SET_BIAS_DISABLE", 0)
        lgpio.gpio_claim_input(handle, pin, flags)
        level = lgpio.gpio_read(handle, pin)
    except Exception as exc:
        return make_result(check, UNAVAILABLE, {"gpio_pin": pin}, f"gpio read failed: {exc}")
    finally:
        if handle is not None:
            try:
                lgpio.gpiochip_free(handle)
            except Exception:
                pass
    active_high = bool(check.get("active_high", True))
    triggered = (level == 1) if active_high else (level == 0)
    observed = {"gpio_pin": pin, "level": level, "pull": pull, "triggered": triggered}
    return make_result(
        check, PASS, observed,
        f"pin {pin} readable (level={level}); presence of a passive switch not verifiable",
    )


CHECKS: dict[str, Callable[[dict, dict], dict]] = {
    "host_identity": check_host_identity,
    "usb_device": check_usb_device,
    "serial_port": check_serial_port,
    "i2c_device": check_i2c_device,
    "i2c_devid": check_i2c_devid,
    "camera": check_camera,
    "pisugar": check_pisugar,
    "command_available": check_command_available,
    "python_module": check_python_module,
    "file_present": check_file_present,
    "disk_space": check_disk_space,
    "host_throttling": check_host_throttling,
    "rosmaster_board": check_rosmaster_board,
    "rosmaster_imu": check_rosmaster_imu,
    "rosmaster_encoders": check_rosmaster_encoders,
    "rosmaster_battery": check_rosmaster_battery,
    "rplidar_health": check_rplidar_health,
    "gpio_input": check_gpio_input,
}


# --------------------------------------------------------------------------- #
# Engine
# --------------------------------------------------------------------------- #
def release_hardware_sessions(ctx: dict) -> None:
    """Close any active serial session opened by active checks."""
    session = (ctx.get("_rosmaster") or {}).get("session")
    if session is None:
        return
    try:
        session.set_auto_report_state(False, forever=False)
    except Exception:
        pass
    try:
        if getattr(session, "ser", None) is not None:
            session.ser.close()
    except Exception:
        pass


def run_checks(config: dict, config_dir: Path) -> dict:
    probe = config.get("probe") or {}
    checks = probe.get("checks") or []
    ctx = {"host": host_info(), "config_dir": config_dir}

    results = []
    for check in checks:
        ctype = check.get("type")
        handler = CHECKS.get(ctype)
        if handler is None:
            results.append(make_result(check, ERROR, None, f"unknown check type '{ctype}'"))
            continue
        try:
            results.append(handler(check, ctx))
        except Exception as exc:  # a check must never crash the run
            results.append(make_result(check, ERROR, None, f"probe exception: {exc}"))

    release_hardware_sessions(ctx)

    counts: dict[str, int] = {}
    required_failures = 0
    for res in results:
        counts[res["status"]] = counts.get(res["status"], 0) + 1
        if res["required"] and res["status"] in HARD_FAIL:
            required_failures += 1

    return {
        "schema_version": 1,
        "probe": {
            "script_version": SCRIPT_VERSION,
            "timestamp_utc": datetime.now(timezone.utc).isoformat(),
            "input_config": str(config_dir / "<config>"),
            "config_name": probe.get("config_name") or config.get("config_name"),
            "mode": probe.get("mode", "discover"),
        },
        "host": ctx["host"],
        "checks": results,
        "summary": {
            "total": len(results),
            "by_status": counts,
            "required_failures": required_failures,
            "deployment_ready": required_failures == 0 and len(results) > 0,
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Read-only, config-driven hardware probe.")
    parser.add_argument("config", help="YAML config with a probe.checks list")
    parser.add_argument("-o", "--output", help="write JSON report to this path")
    parser.add_argument("--compact", action="store_true", help="compact JSON (no indent)")
    args = parser.parse_args()

    config_path = Path(args.config).expanduser().resolve()
    if not config_path.exists():
        sys.stderr.write(f"ERROR: config not found: {config_path}\n")
        return 2

    with open(config_path, encoding="utf-8") as handle:
        config = yaml.safe_load(handle) or {}

    report = run_checks(config, config_path.parent)
    report["probe"]["input_config"] = str(config_path)

    text = json.dumps(report, indent=None if args.compact else 2, default=str)
    if args.output:
        Path(args.output).write_text(text + "\n", encoding="utf-8")
        sys.stderr.write(f"wrote {args.output}\n")
    else:
        print(text)

    return 0 if report["summary"]["deployment_ready"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
