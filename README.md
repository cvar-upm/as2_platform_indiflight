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

## pi-protocol (indiflight high-rate IMU, up to 2kHz)

A second, independent node (`as2_platform_indiflight_pi_protocol_node`, launched alongside the
main platform node) reads indiflight's `pi-protocol` telemetry channel and publishes
`sensor_msgs/msg/Imu` on `imu_high_rate`. This is a separate channel from MSP, capable of much
higher rates (up to 2kHz, scheduler load permitting) than MSP's ~100Hz polling cap.

**Wiring**: on the FC, `FUNCTION_TELEMETRY_PI` must be assigned to a UART that is physically
wired to a *second* serial port on the host (distinct from the MSP UART), at 921600 baud — e.g.
via the CLI: `serial <n> 262144 115200 57600 0 921600` (see indiflight's own `BTFL_cli` example
configs for the exact flag values on your target). Set `pi_protocol.device` /
`pi_protocol.baudrate` in `config/platform_config_file.yaml` to match, and
`pi_protocol.enable: true` to turn the node on (it defaults to disabled since the device path is
deployment-specific).

**Build-time dependency**: the pi-protocol wire format is code-generated from
[tudelft/pi-protocol](https://github.com/tudelft/pi-protocol)'s YAML message definitions via
Jinja2 templates, at build time. This requires `python3` plus the packages in that repo's
`python/requirements.txt` (Jinja2, PyYAML, semver) to be installed on the build machine:
```
pip install -r <build-dir>/thirdparty/pi-protocol/python/requirements.txt
```
(the path only exists after the first `colcon build` has fetched the dependency — install these
packages before building, or re-run the build after installing them if it fails on the codegen
step).

