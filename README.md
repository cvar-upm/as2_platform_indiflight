# as2_platform_indiflight

[Aerostack2](https://aerostack2.github.io/) Aerial platform for indiflight drones.

## Command/state link: pi-protocol, not MSP

This node's command and state link runs entirely over indiflight's `pi-protocol` channel, not
MSP - there is no `msp::FlightController` connection in this node at all. `pi_protocol.enable`
defaults to `true` and a failed connection is **fatal** (the node throws on startup), since it's
the only link this node has to the FC.

`pi-protocol` carries three things:
- **`RC_OVERRIDE`** (host→FC, sent every `sendCommand()` tick): roll/pitch/yaw/throttle, in the
  same 1000-2000 pulse convention MSP used. This only takes effect on the FC while indiflight's
  `PI OVERRIDE` box mode is active - it overrides *only* those 4 channels.
- **`PI_STATUS`** (FC→host, ~50Hz): armed / `PI OVERRIDE` active / rx-link-valid flags. Drives
  this node's `setArmingState()`/`setOffboardControl()` calls directly (`onPiStatus()`) -
  `PI_OVERRIDE_ACTIVE` *is* this platform's notion of offboard.
- **`BATTERY`** (FC→host, ~50Hz): pack voltage/current/cell count, for the voltage-aware thrust
  map.

**ARM, offboard and kill-switch are physical-radio-only on this platform.** A safety pilot's
transmitter is the base receiver underneath indiflight's `PI OVERRIDE` mode (mirrors Betaflight's
own `MSP OVERRIDE`/`BOXMSPOVERRIDE` pattern, just sourced from pi-protocol instead of MSP) - ARM
and the `PI OVERRIDE` AUX switch live there permanently, outside pi-protocol's reach.
`ownSetArmingState()`, `ownSetOffboardControl()` and `ownKillSwitch()` are therefore no-ops that
log a warning: this node can *read* FC state via `PI_STATUS`, it can't *set* arm/offboard/kill.

MSP is not used by this node. A handful of MSP-based telemetry callbacks
(`onAltitude`/`onAttitude`/`onMotor`/`onRc` and their `*_hz` params) are still declared in the
source as dormant placeholders for pi-protocol equivalents that don't exist yet, but nothing
wires them up - they're unreachable dead code today.

### PI OVERRIDE firmware setup
On the FC, assign an AUX switch to the `PI OVERRIDE` box mode and set
`pi_override_channels_mask` (defaults to `15`, i.e. roll/pitch/yaw/throttle) via CLI. Flipping
that switch off instantly returns stick control to the physical radio.

## pi-protocol high-rate telemetry (up to 2kHz, synchronized)

The platform node also reads indiflight's `pi-protocol` telemetry channel on a dedicated
background thread. The wire format is a single synchronized bundle per sample — `EKF_INPUTS`
(accel + gyro rates + all 4 motor speeds, one snapshot, one `time_us`, fixed-point encoded for
bandwidth) — republished here as two separate ROS topics once parsed, since the synchronization
only needs to happen on the wire, not in how it's exposed to consumers:
- `sensor_msgs/msg/Imu` on `imu_high_rate`, and also fed into AS2's standard
  `sensor_measurements/imu` topic (via the `imu` sensor wrapper) for the state estimator.
- `sensor_msgs/msg/JointState` on `motor_speed_high_rate` — measured motor angular speeds
  (rad/s, `velocity` field). Indexed in **Betaflight's own mixer output order**
  (`[RR, FR, RL, FL]`) — this is *not* the `indi_controller`/simulator convention used elsewhere
  in the wider workspace (`[FR, RR, RL, FL]`); permute if you need to match that.

Both topics always share the exact same sample instant and timestamp — unlike sending IMU and
motor data as two independently-timed messages, this is a real synchronization guarantee, not
just accurate timestamping of two separately-sampled streams. That matters for anything that
numerically differentiates gyro or evaluates a model at the current motor speed (e.g. an
offboard INDI controller): it needs every input from the same instant, not just correctly
time-stamped inputs from different instants.

**Timestamps**: `header.stamp` on both topics is *not* raw host-arrival time — the FC's
`time_us` (a monotonic microsecond counter since FC boot, in an epoch unrelated to the host
clock) is converted to a host-clock timestamp via a continuously-adapting offset tracker
(`PiProtocolClockSync`, sliding-window minimum-offset / "clock filter" method), self-correcting
for clock drift over long runs. The raw `time_us` is preserved too, published alongside as
`sensor_msgs/msg/TimeReference` on `pi_protocol/time_reference` (`time_ref` = raw FC time,
`source` = `"indiflight_fc_micros"`) — one shared topic, since both `imu_high_rate` and
`motor_speed_high_rate` now always come from the same synchronized sample.

**Wiring**: on the FC, `FUNCTION_TELEMETRY_PI` must be assigned to a UART that is physically
wired to a serial port on the host, at 921600 baud — e.g. via the CLI:
`serial <n> 262144 115200 57600 0 921600` (see indiflight's own `BTFL_cli` example configs for
the exact flag values on your target). Set `pi_protocol.device` / `pi_protocol.baudrate` in
`config/platform_config_file.yaml` to match the port your deployment actually uses.

**Build-time dependency**: the pi-protocol wire format is code-generated from
[fjanguita/pi-protocol](https://github.com/fjanguita/pi-protocol)'s YAML message definitions via
Jinja2 templates, at build time. This requires `python3`
plus the packages in that repo's `python/requirements.txt` (Jinja2, PyYAML, semver) to be
installed on the build machine:
```
pip install -r <build-dir>/thirdparty/pi-protocol/python/requirements.txt
```
(the path only exists after the first `colcon build` has fetched the dependency — install these
packages before building, or re-run the build after installing them if it fails on the codegen
step).

