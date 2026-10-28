# CGTD070 remote-control integration

## Command-frame trace without a gimbal

Enable the trace after starting the module:

    cgtd070 trace on

Each completed non-idle serial frame is printed as four indexed lines. Idle
00H frames still transmit at 25 Hz but are not printed. Operate AUX1/AUX2 and
check that command byte `TX[02]` is `93`, `71`, or `3b`. Disable with:

    cgtd070 trace off

The message means all 44 bytes were accepted by the PX4 serial driver. It
validates RC mapping, edge detection, command queuing, protocol packing, and
the software write path. It cannot by itself prove the UART pin waveform; use
a USB-TTL adapter or logic analyser for that final hardware check.

This version adds RC control only in the PX4 adapter. The host-tested protocol
core (`library/`) and serial transport (`drivers/serial/`) are unchanged from
the verified baseline.

## Fixed PX4 interface

The module subscribes to `manual_control_setpoint`, not raw `input_rc` channel
indices. The transmitter controls can use any unused physical RC channels;
PX4 parameters map them to fixed AUX fields:

```text
RC_MAP_AUX1 = channel used by the three-position switch
RC_MAP_AUX2 = channel used by the momentary button
```

## Command mapping

| Input | Stable value | One-shot command |
|---|---:|---|
| AUX1 | below -0.5 | cancel, 3BH |
| AUX1 | -0.5 through +0.5 | safe position, no command |
| AUX1 | above +0.5 | track, 93H parameter 1 = 0 |
| AUX2 | crosses to above +0.5 | center, 71H |

Each transition must remain stable for 100 ms. A command replaces one 25 Hz
idle frame; subsequent frames return to idle 00H. Holding a switch or button
does not repeat a command.

## Safety behavior

- Startup records current control positions without commanding.
- RC recovery after loss also records positions without commanding.
- Invalid, stale (over 500 ms), non-RC, or NaN AUX data is ignored.
- AUX1 middle is deliberately command-free.
- Existing NSH commands (`track`, `center`, `cancel`) remain available.

## Hardware setup and verification

1. In the Tango 2 model Mixer, map the three-position switch and momentary
   button to two unused transmitted channels.
2. Confirm raw channels using `listener input_rc 20`.
3. Set `RC_MAP_AUX1` and `RC_MAP_AUX2` to those channel numbers.
4. Verify `listener manual_control_setpoint 20` reports AUX1 near -1/0/+1 and
   AUX2 changing from low to high when pressed.
5. With propulsion disabled, use `cgtd070 status` and a serial monitor to
   verify one 3BH/93H/71H frame per deliberate transition.

If directions are reversed, reverse the corresponding Tango 2 Mixer output.
