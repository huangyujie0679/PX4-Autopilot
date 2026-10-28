# CGTD070 uORB interface

`cgtd070_status` is intentionally a small guidance-facing topic. The protocol
core retains the complete decoded `cgtd070::Feedback`; the PX4 adapter selects
only four telemetry values for uORB.

## Message definition

```text
uint64 timestamp
float32 los_az_rate
float32 los_el_rate
float32 gimbal_roll
float32 gimbal_pitch
```

`gimbal_roll` is the user-requested name for the protocol field documented as
the gimbal azimuth/frame angle. It is not the protocol's gimbal roll-frame
angle. `gimbal_pitch` is the pitch frame angle. Units remain degrees and
degrees per second.

## Consumer example

```cpp
#include <uORB/Subscription.hpp>
#include <uORB/topics/cgtd070_status.h>

uORB::Subscription cgtd070_sub{ORB_ID(cgtd070_status)};
cgtd070_status_s status{};

if (cgtd070_sub.update(&status)) {
    const float los_az = status.los_az_rate;
    const float los_el = status.los_el_rate;
    const float gimbal_az = status.gimbal_roll;
    const float gimbal_el = status.gimbal_pitch;
}
```

Console verification:

```sh
listener cgtd070_status 1
uorb top cgtd070_status
```
