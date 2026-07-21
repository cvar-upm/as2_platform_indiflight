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

