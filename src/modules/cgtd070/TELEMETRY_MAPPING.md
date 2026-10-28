# Flight telemetry mapping (control-frame bytes 8..37)

All byte numbers are one-based protocol byte numbers. Multi-byte numeric
values are little-endian. Float values are IEEE-754 float32.

| Bytes | Protocol value | PX4 source | Invalid/missing behavior |
|---|---|---|---|
| 8..9 | pitch, 0.01 deg | `vehicle_attitude.q` | zero after 0.5 s |
| 10..11 | roll, 0.01 deg | `vehicle_attitude.q` | zero after 0.5 s |
| 12..13 | heading, 0.01 deg | `vehicle_attitude.q` | zero after 0.5 s |
| 14..20 | UTC | `sensor_gps.time_utc_usec` | zero |
| 21..24 | longitude, float32 deg | `sensor_gps.longitude_deg` | zero |
| 25..28 | latitude, float32 deg | `sensor_gps.latitude_deg` | zero |
| 29 | satellites, max protocol value 24 | `sensor_gps.satellites_used` | zero |
| 30..33 | MSL altitude, float32 m | `sensor_gps.altitude_msl_m` with 3D fix | zero |
| 34..35 | battery, 0.1 V | `battery_status.voltage_v` | zero |
| 36..37 | above Home, 0.1 m | global altitude minus valid Home altitude | zero |

The adapter uses non-blocking `uORB::Subscription::update()`. It constructs a
new zeroed snapshot on every TX slot and never waits for any source topic.
Thus GPS, estimator, Home or battery failure cannot stop the 25 Hz frame loop.
