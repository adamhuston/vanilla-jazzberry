# Vanilla Jazzberry sensor bringup

Reusable device plugins live in Jazzy's `rov2_sensors` package under the
[`bot` submodule](../bot/mx_jazzhands/src/rov2_sensors/README.md).
This directory owns **robot-specific wiring, mounting, calibration, and safety
policy**, not forks of those sensor drivers.

## Build and run

Use native ROS 2 Jazzy in **WSL Ubuntu-24.04**, not Docker Desktop. Isaac Sim
remains native Windows. The wrapper reuses the canonical
`bot/mx_jazzhands/scripts/build_dev.sh` and overlay, adding `vanilla_bringup`
to package discovery; it does not build or source the older `~/jazzy` checkout.

From PowerShell:

```powershell
wsl -d Ubuntu-24.04 -- bash /mnt/c/projects/vanilla-jazzberry/vanilla_bot/scripts/build.sh --packages-up-to vanilla_bringup
wsl -d Ubuntu-24.04 -- bash /mnt/c/projects/vanilla-jazzberry/vanilla_bot/scripts/run_sensors.sh
```

The launch loads **only** `Rplidar` and `Adxl345`; it loads no actuator or brain.
Starting acquisition spins the LiDAR's scan motor, but cannot command rover
wheel motion. Core activation is acquisition activation, **not SafetyCzar
activation**. Missing devices yield visible faults and no fabricated data.
`required_plugins` requires plugin loading/configuration, while fresh input
and mounting checks are explicitly future SafetyCzar activation requirements.

On the Pi, install a unique `/dev/rplidar` udev symlink for the A1's CP2102
(`10c4:ea60`), or override `config` with a robot-local parameter file. Do not
run the active hardware probe while these plugins own their devices.
The ADXL345 uses `/dev/i2c-3`, address `0x53`, GPIO23/24, as recorded in the
[hardware contract](probe/robot_hardware.yaml). No Yahboom/serial dependency
exists in the accelerometer plugin.

For Pi deployment, build natively on the target (with a compatible ROS 2
installation) using the same Vanilla build wrapper from its local checkout.
The generic `build_pi.sh` now includes `rov2_sensors`, but use the Vanilla
wrapper to discover `vanilla_bringup` as well. Do not copy this workstation's
x86_64 WSL binaries to the ARM Pi. ARM building and real device operation
have not been verified in this change.

Data:

- `/scan`: `sensor_msgs/LaserScan`, `lidar_link`, 250 ms acquisition-age budget.
- `/safety/accel`: `sensor_msgs/Imu`, `safety_accel_link`, 100 ms budget,
  acceleration only, including gravity. Acquisition/publication is 100 Hz;
  FIFO batches preserve short transients rather than selecting only one sample
  per core tick.
- `/sensors/rplidar/health`, `/sensors/adxl345/health`: reliable
  `diagnostic_msgs/DiagnosticArray`, 10 Hz even when no data is available.
- `/diagnostics`: the same structured per-sensor health plus core diagnostics.

### Mounting and calibration

Static TF publishers are disabled by default. Replace provisional XYZ/RPY in
[`mounts.launch.xml`](vanilla_bringup/launch/mounts.launch.xml), then launch with
`mounts_verified:=true`. Do not publish dummy mounting as robot truth.
Coordinate conventions are x forward, y left, z up; verify LiDAR angular zero
and accelerometer axes against physical reference targets. SafetyCzar must
refuse safety-ready activation without the required transforms.

Collect and report:

1. LiDAR center XYZ and roll/pitch/yaw relative to `base_link`, and reference
   obstacle directions (front, left, rear, right).
2. ADXL345 XYZ and roll/pitch/yaw, including breakout axis markings.
3. Six-face stationary acceleration readings to estimate per-axis bias/scale.
4. Stationary acceleration noise variance at the configured range/rate.
5. Actual scan frequency, first-ray age/jitter, partial-return coverage, and
   disconnect/recovery behavior.
6. ADXL345 FIFO behavior under load, short transient capture, and bus-fault
   shutdown/recovery latency.

Bias `[0,0,0]`, scale `[1,1,1]`, noise variance `0.01`, and mounting numbers are
provisional test values, not measured physical facts. Update
[`sensors.yaml`](vanilla_bringup/config/sensors.yaml) and the hardware contract
after calibration.

### Safety policy, not implemented governance

The [policy](probe/safety_czar.yaml) records:

- LiDAR loss/staleness: stop.
- ADXL345 loss/staleness while moving: **degrade**, as requested. Intersect the
  current envelope with the existing `test_low_speed` profile (25% limits);
  never increase an already tighter limit. Recover only on fresh valid evidence.
- Both inputs must be fresh before **initial safety-ready activation**.

These are consumer obligations. Sensor health publication does not implement
SafetyCzar, enforce a velocity reduction, replace its command gate, or authorize
motion. The original drivetrain-only hardware profile remains separate.

### Isaac Sim compatibility

Keep the established domain 0 / `rmw_fastrtps_cpp` / loopback-unicast peer
configuration from the [connectivity runbook](../sim/docs/connectivity_runbook.md).
`run_sensors.sh` preserves a supplied `FASTRTPS_DEFAULT_PROFILES_FILE`; no
network/firewall configuration is changed.

Hardware plugins should **not** run as duplicate publishers in a simulation
session. Future simulator `/scan` and `/safety/accel` publishers must satisfy
the same message/frame/freshness contract; sensor-frame TF belongs to that
simulated robot. Implementing those simulated sensors is separate work.
