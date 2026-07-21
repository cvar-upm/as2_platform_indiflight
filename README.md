# as2_platform_indiflight

[Aerostack2](https://aerostack2.github.io/) Aerial platform for betaflight/indiflight drones.

## Betaflight Information

This code is tested with betaflight 4.5.0 and uses the MSP protocol to communicate with the drone.
It also works unmodified against [indiflight](https://github.com/tudelft/indiflight) (a Betaflight
4.4 research fork): indiflight preserves the same MSP message set and still reports itself as
`FC_VARIANT = "BTFL"`, so no protocol-level changes are required for this node to connect. Note that
`MSP_SET_ARMING_DISABLED` behaves differently on indiflight - it disarms if currently armed, but no
longer persistently blocks future arming - so don't rely on it as a standing safety interlock.

```
cd aerostack2_ws/src
git clone git@github.com:CVAR-A2RL/msp.git
as2 build
```

### Betaflight configuration
Channels:
    - 0: Roll
    - 1: Pitch
    - 2: Throttle
    - 3: Yaw
    - 4: Arm
    - 5: Mode
    - 6: Aux1
    - 7: Aux2

## pi-protocol (indiflight high-rate telemetry: IMU @ 2kHz, motor speeds @ 1kHz)

The platform node also reads indiflight's `pi-protocol` telemetry channel on a dedicated
background thread and publishes:
- `sensor_msgs/msg/Imu` on `imu_high_rate` — up to 2000Hz.
- `sensor_msgs/msg/JointState` on `motor_speed_high_rate` — measured motor angular speeds
  (rad/s, `velocity` field), up to 1000Hz. Indexed in **Betaflight's own mixer output order**
  (`[RR, FR, RL, FL]`) — this is *not* the `indi_controller`/simulator convention used elsewhere
  in the wider workspace (`[FR, RR, RL, FL]`); permute if you need to match that.

This is a separate channel from MSP, capable of much higher rates than MSP's ~100Hz polling cap.
A failure to connect to the pi-protocol UART is logged but is **non-fatal** — the platform node
keeps running MSP/flight control regardless, it just won't publish either high-rate topic.

**Timestamps**: `header.stamp` on both topics is *not* raw host-arrival time — the FC's
`time_us` (a monotonic microsecond counter since FC boot, in an epoch unrelated to the host
clock) is converted to a host-clock timestamp via a continuously-adapting offset tracker
(`PiProtocolClockSync`, sliding-window minimum-offset / "clock filter" method), self-correcting
for clock drift over long runs. The raw `time_us` is preserved too, published alongside as
`sensor_msgs/msg/TimeReference` on `imu_high_rate/time_reference` and
`motor_speed_high_rate/time_reference` (`time_ref` = raw FC time, `source` =
`"indiflight_fc_micros"`), correlated against the same corrected `header.stamp`.

**Wiring**: on the FC, `FUNCTION_TELEMETRY_PI` must be assigned to a UART that is physically
wired to a *second* serial port on the host (distinct from the MSP UART), at 921600 baud — e.g.
via the CLI: `serial <n> 262144 115200 57600 0 921600` (see indiflight's own `BTFL_cli` example
configs for the exact flag values on your target). Set `pi_protocol.device` /
`pi_protocol.baudrate` in `config/platform_config_file.yaml` to match, and
`pi_protocol.enable: true` to turn it on (it defaults to disabled since the device path is
deployment-specific).

**Build-time dependency**: the pi-protocol wire format is code-generated from
[fjanguita/pi-protocol](https://github.com/fjanguita/pi-protocol)'s (`motor-telemetry-support`
branch) YAML message definitions via Jinja2 templates, at build time. This requires `python3`
plus the packages in that repo's `python/requirements.txt` (Jinja2, PyYAML, semver) to be
installed on the build machine:
```
pip install -r <build-dir>/thirdparty/pi-protocol/python/requirements.txt
```
(the path only exists after the first `colcon build` has fetched the dependency — install these
packages before building, or re-run the build after installing them if it fails on the codegen
step).

