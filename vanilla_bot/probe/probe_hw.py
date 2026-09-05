#!/usr/bin/env python3
"""Config-driven, read-only hardware probe for Vanilla Jazzberry.

Reads a YAML config that carries a ``probe.checks`` list, runs each read-only
check against the Raspberry Pi host, and emits a single JSON report.

Run this ON THE PI (Raspberry Pi OS / Debian). It is read-only: it lists and
queries devices and never configures, writes, or moves anything. Motion and
actuator behavior are deliberately out of scope for this tool.

Usage:
    python3 probe_hw.py robot_hardware.yaml
    python3 probe_hw.py robot_hardware.yaml --output hardware_probe.json
    python3 probe_hw.py safety_czar.yaml

The same script accepts either YAML file; each file declares the checks that
matter to its concern. Run it once per file.

Dependencies: PyYAML (`pip install pyyaml`). Everything else is stdlib. Optional
host tools (i2c-tools, libcamera, Rosmaster_Lib) are used if present and are
reported as ``unavailable`` when missing, never fatal.
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
}


# --------------------------------------------------------------------------- #
# Engine
# --------------------------------------------------------------------------- #
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
