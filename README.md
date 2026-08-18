# as2_platform_indiflight

[Aerostack2](https://aerostack2.github.io/) Aerial platform for indiflight drones.

**Both control modes are supported by the same node and the same configuration.** Nothing
is switched at build time or by picking a different `control_modes.yaml`: the platform
advertises `BODY_RATES`, `POSITION`, `HOVER` and `UNSET`, and executes whichever the motion
controller negotiates. Likewise every pi-protocol message is handled, and each FC simply
triggers the subset it actually sends. Which control mode is negotiated and which messages
an FC streams are independent axes:

| Control mode | Uplink |
|---|---|
| `BODY_RATES` | `RC_OVERRIDE` rate/thrust pulses (loop closed host-side) |
| `POSITION` | `POS_SETPOINT` (loop closed on the FC, needs `USE_LOCAL_POSITION`) |
| `HOVER` | nothing; the FC holds its last setpoint |

| Message | Direction | Consumed as |
|---|---|---|
| `EKF_INPUTS` | FC→host | `sensor_measurements/imu` + `motor_angular_speed` |
| `PI_STATUS` | FC→host | platform armed/offboard |
| `BATTERY` | FC→host | `sensor_measurements/battery` |
| `AUX` | FC→host | `debug/aux` (raw switch values) |
| `RC_OVERRIDE` | host→FC | the `BODY_RATES` command |
| `POS_SETPOINT` | host→FC | the `POSITION` command |
| `EXTERNAL_POSE` | host→FC | measurement for the FC's onboard EKF |

The only per-deployment settings are `external_pose.enable` (does this FC's firmware consume
`EXTERNAL_POSE`?) and `num_rotors`. Note that the
`EXTERNAL_POSE` uplink is **not** tied to `POSITION`: it runs in every mode, so the FC's EKF
is already converged when a `set_mode` hands control over mid-flight.

The pi-protocol table compiled into this node is taken verbatim from the pinned pi-protocol
checkout, the same table both firmware forks carry: `MOTOR` dropped (its rotor speeds duplicate
the ones `EKF_INPUTS` already bundles with the gyro), and `EKF_INPUTS` carrying six rotor
slots so one host binary serves quad and hex. **The FC must be flashed with a
firmware built against it**, otherwise nothing parses. Mis-framed packets that slip the 8-bit
CRC (~1/256) are rejected by an FC-timestamp plausibility gate (`FcStampGate`) before
reaching any callback.

## Command/state link: pi-protocol

The command and state link runs entirely over indiflight's `pi-protocol` channel. It is the only
link to the FC, so a failed connection is **fatal**: the node throws on startup rather than
running without commands or state.

`pi-protocol` carries, depending on the firmware build:
- **`RC_OVERRIDE`** (host→FC, sent every `sendCommand()` tick): roll/pitch/yaw/throttle, in the
  same 1000-2000 pulse convention MSP used. This only takes effect on the FC while indiflight's
  `PI OVERRIDE` box mode is active - it overrides *only* those 4 channels.
- **`PI_STATUS`** (FC→host, ~50Hz): armed / `PI OVERRIDE` active / rx-link-valid flags, which
  drive this platform's armed/offboard state (`onPiStatus()`). The FC decodes its own switch
  bands, so none of them are duplicated host-side.
- **`AUX`** (FC→host, ~50Hz): the 14 raw aux channel values, republished on `debug/aux`. Not
  used for arm/offboard; it is there to check switch bands against the radio and as the hook
  for any host-side switch behaviour.
- **`BATTERY`** (FC→host, ~50Hz): pack voltage/current/cell count, for the voltage-aware thrust
  map.
- **`EKF_INPUTS`** (FC→host, high rate): inertial data + rotor speeds, see below.
- **`POS_SETPOINT`** and **`EXTERNAL_POSE`** (host→FC): the `POSITION` mode uplink, see below.

**ARM, offboard and kill-switch are physical-radio-only on this platform.** A safety pilot's
transmitter is the base receiver underneath indiflight's `PI OVERRIDE` mode (mirrors Betaflight's
own `MSP OVERRIDE`/`BOXMSPOVERRIDE` pattern, just sourced from pi-protocol instead of MSP) - ARM
and the `PI OVERRIDE` AUX switch live there permanently, outside pi-protocol's reach.
`ownSetArmingState()`, `ownSetOffboardControl()` and `ownKillSwitch()` are therefore no-ops that
log a warning: this node can *read* FC state via `PI_STATUS`, it can't *set* arm/offboard/kill.

### PI OVERRIDE firmware setup
On the FC, assign an AUX switch to the `PI OVERRIDE` box mode and set
`pi_override_channels_mask` (defaults to `15`, i.e. roll/pitch/yaw/throttle) via CLI. Flipping
that switch off instantly returns stick control to the physical radio.

## pi-protocol high-rate telemetry

The platform node reads indiflight's `pi-protocol` telemetry channel on a dedicated background
thread. `EKF_INPUTS` is a single synchronized bundle per sample — accel + gyro rates + up to
six rotor speeds, one snapshot, one `time_us`, fixed-point encoded for bandwidth — republished
as two ROS topics (`num_rotors` selects how many of the six slots are published):
- `sensor_msgs/msg/Imu` on `sensor_measurements/imu` (via the `imu` sensor wrapper) — also the
  topic AS2's state estimator consumes.
- `sensor_msgs/msg/JointState` on `sensor_measurements/motor_angular_speed` — measured motor
  angular speeds (rad/s, `velocity` field). Indexed in **Betaflight's own mixer output order**
  (quad X: `[RR, FR, RL, FL]`, hex X adds `[MR, ML]`) — this is *not* the
  `indi_controller`/simulator convention used elsewhere in the wider workspace
  (`[FR, RR, RL, FL]`); permute if you need to match that.

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
`sensor_msgs/msg/TimeReference` on `debug/platform/og_timestamp` (`time_ref` = raw FC time,
`source` = `"indiflight_fc_micros"`) — one shared topic, since both `sensor_measurements/imu` and
`sensor_measurements/motor_angular_speed` now always come from the same synchronized sample.

**Wiring**: on the FC, `FUNCTION_TELEMETRY_PI` must be assigned to a UART that is physically
wired to a serial port on the host, at 921600 baud — e.g. via the CLI:
`serial <n> 262144 115200 57600 0 921600` (see indiflight's own `BTFL_cli` example configs for
the exact flag values on your target). Set `pi_protocol.device` / `pi_protocol.baudrate` in
`config/platform_config_file.yaml` to match the port your deployment actually uses.

## Layout

The link to the FC is a subproject with no ROS dependency, so a change to the communication is
contained in one directory and builds and tests on its own:

```
CMakeLists.txt              ROS package: node, platform, behaviours
include/ src/ tests/        the AS2 platform and the frame conversions
pi_protocol_interface/      the wire, ROS-free
├── CMakeLists.txt          checkout, codegen, library, tests
├── include/ src/           Client (serial, framing) and ClockSync
├── tests/                  codec round-trip and clock filter, hardware-free
└── pi-protocol_lib/        the pi-protocol checkout, gitignored
```

The message table is **not** kept here: it comes from the checkout in `pi-protocol_lib/`,
the same repo the firmware builds against. That directory is cloned on first build if
missing, so dropping the other vehicle's pi-protocol there makes this node speak its table
with no CMake change. A checkout carrying a different table fails the build at the
`static_assert`s in `pi_protocol_interface/tests/parser_gtest.cpp`, not silently at runtime.

The parent links a single target, `pi_protocol_interface`. Building it standalone needs
nothing but CMake and a compiler:

```bash
cmake -S pi_protocol_interface -B build/pi_protocol && cmake --build build/pi_protocol
```

**Build-time dependency**: the codec is generated from
[cvar-upm/pi-protocol](https://github.com/cvar-upm/pi-protocol)'s YAML definitions via Jinja2
templates. This needs `python3` plus that repo's `python/requirements.txt` (Jinja2, PyYAML,
semver):
```
pip install -r pi_protocol_interface/pi-protocol_lib/pi-protocol/python/requirements.txt
```
(the path exists after the first build has fetched the dependency — install the packages
beforehand, or re-run the build once they are there).

## `POSITION` mode: onboard position control

A firmware built with `USE_LOCAL_POSITION` + `USE_EKF` runs its own EKF and position
controller on the FC. In `POSITION` mode this node
acts as a thin bridge: it feeds the FC's EKF an external pose and streams position setpoints,
instead of closing any loop host-side.

Nothing changes in the launch command — only the config:

```yaml
external_pose:
  enable: true
  rate: 100.0          # both topics empty -> poll the earth->base_link TF
  # pose_topic: "/drone0/mocap_pose"          # or a PoseStamped source
  # mocap_topic: "/mocap/rigid_bodies"        # or a RigidBodies source
  # rigid_body_name: "hexacopter"             # required with mocap_topic
```

and the AS2 motion controller must run with `use_bypass: true`, so `POSITION` references pass
straight through to `actuator_command/pose` (the geometric controller only outputs `BODY_RATES`).
`SPEED`/`TRAJECTORY` behaviors will not negotiate a mode against this platform; position-only
missions are the supported envelope.

**Feeding the FC EKF is independent of the control mode.** `external_pose.enable` starts a timer
that runs whatever mode is active, including `BODY_RATES`. That is deliberate: the FC EKF needs about
2 s of continuous measurements to converge (`EKF_CONVERGE_TIME_US`) before `POSITION_MODE` can
engage, so a vehicle flying in `BODY_RATES` with the feed running can be handed over with a `set_mode`
and take it immediately, instead of waiting mid-air.

**Data path** (all conversions in `include/as2_platform_indiflight/conversions.hpp`):
- latest `earth`→`base_link` TF → ENU→NED + FLU→FRD quaternion → `EXTERNAL_POSE`. TF is polled
  (`external_pose.rate`), not subscribed, and only a transform whose `header.stamp` advanced is
  forwarded — polling above the estimator's rate therefore costs nothing and picks up each new
  estimate as soon as it exists. **Velocity is sent as zero on purpose**: the FC EKF measurement
  vector is position + quaternion only (`flight/ekf.c`'s `ekf_Z[0..2]` from `posMeasNed.pos`,
  `[3..6]` from `posMeasNed.quat`); the wire velocity reaches the blackbox log and nothing else.
  That is what makes TF, which carries no velocity, a complete source here.
- `actuator_command/pose` (converted to `"earth"` here if needed) → ENU→NED, yaw → NED degrees →
  `POS_SETPOINT`. Velocity feed-forward is sent as zero too, for a different reason: AS2's
  POSITION twist is a speed *limit*, not a feed-forward. `HOVER` transmits nothing — the FC keeps
  flying to its last setpoint.
- Both messages are stamped with the **last FC tick seen on the downlink** (the FC compares
  `time_us` against its own `micros()` and silently discards anything stale); `EXTERNAL_POSE` is
  additionally skipped until the tick advances (the FC requires strictly increasing stamps), so
  its effective rate is bounded by the downlink rate. The FC EKF rejects measurements older than
  100 ms — verify the downlink rate on the bench (`ros2 topic hz sensor_measurements/imu`)
  before flying.

**Arm / offboard** come from `PI_STATUS`, i.e. the FC's own state: `armed` is
`ARMING_FLAG(ARMED)` and `offboard` is the FC actually obeying this node. Both are driven by
physical radio switches the FC decodes itself, and `POSITION` additionally requires a converged
FC EKF. `debug/aux` publishes the raw channel values if you need to see what the radio sends.

**Bench checklist before the first flight** (this is what separates flying from crashing):
1. Vehicle disarmed, state estimator running: check `sensor_measurements/imu` flows and measure
   its rate, and that `earth`→`base_link` exists in TF (`ros2 run tf2_ros tf2_echo earth
   drone0/base_link`) — no TF, no `EXTERNAL_POSE`.
2. In the FC blackbox/OSD, verify `posEstNed` follows the real vehicle with the correct sign on
   all three axes and that the estimated heading matches reality (North = +x NED).
3. Compare the emitted `POS_SETPOINT` against the mocap pose: same axes, same signs, same datum.
4. Flip BOXPOSCTL and verify the FC EKF converges (~2 s) and `POSITION_MODE` engages.
5. With ARM + BOXPOSCTL up, confirm the platform agrees: `ros2 topic echo
   /drone0/platform/info` must show `armed: true, offboard: true`, and the FC must be
   receiving setpoints (blackbox `posSpNed`, or the parse-error count staying flat).

