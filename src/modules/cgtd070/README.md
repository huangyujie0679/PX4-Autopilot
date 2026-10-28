# CGTD070 PX4 v1.17 independent Task module

This is the Task-based adapter. The older Work Queue project at
`outputs/cgtd070_px4_app` is not modified.

## Runtime architecture

- UART: 115200, 8N1, non-blocking; open/read/write/close run in one PX4 Task.
- TX: one complete 44-byte frame every 40 ms (25 Hz).
- RX: arbitrary stream chunks are buffered and decoded as 64-byte feedback.
- Memory: fixed buffers and no STL containers, exceptions or RTTI. The PX4
  module object is allocated once by `ModuleBase` and freed only at stop.

## Flight telemetry in the 44-byte control frame

The PX4 adapter reads the following topics without blocking:

- `vehicle_attitude`: pitch, roll and heading.
- `sensor_gps`: UTC, longitude, latitude, satellites and MSL altitude.
- `battery_status`: pack voltage.
- `vehicle_global_position` plus `home_position`: altitude above Home.

Every 40 ms a new zero-initialized telemetry snapshot is built. Only fresh,
valid topics overwrite their fields. Missing, stale or invalid data therefore
becomes zero exactly as required by protocol V2.12, while TX continues at
25 Hz. Joystick bytes 38..42 remain zero.

## Feedback and uORB

The protocol core still decodes every feedback field. Only four values are
published to the minimal `cgtd070_status` topic:

- `los_az_rate`: target azimuth LOS rate, deg/s.
- `los_el_rate`: target elevation LOS rate, deg/s.
- `gimbal_roll`: protocol gimbal azimuth/frame angle, deg (requested name).
- `gimbal_pitch`: protocol gimbal pitch frame angle, deg.

JSON and unknown frames are decoded and counted but are not published to this
guidance topic.

## Install and build

Copy the directory to `src/modules/cgtd070`, then replace the root message:

```sh
cd ~/PX4-Autopilot-1.17.0
cp src/modules/cgtd070/msg/Cgtd070Status.msg msg/Cgtd070Status.msg
grep -n 'Cgtd070Status.msg' msg/CMakeLists.txt
make cuav_fmu-v6x_cgtd070
```

The board variant must contain `CONFIG_MODULES_CGTD070=y`.

## Bench/SITL commands

```text
cgtd070 start -d /dev/ttyS3
cgtd070 status
listener cgtd070_status 1
cgtd070 track
cgtd070 center
cgtd070 cancel
cgtd070 stop
```

For SITL, replace `/dev/ttyS3` with `/tmp/cgtd070_px4`.
