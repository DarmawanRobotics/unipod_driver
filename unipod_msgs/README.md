# unipod_msgs

Messages and services of [unipod_driver](../README.md).

## Messages

| | |
|---|---|
| `GimbalAttitude` | yaw, pitch, roll (°) and their rates (°/s) |
| `GimbalStatus` | connected, zoom and zoom max, video mode of both streams, palette, gain, firmware |
| `ThermalData` | max / min temperature (°C) and where they are |

## Services

| | |
|---|---|
| `SetGimbalAngle` | absolute yaw / pitch |
| `Recenter` | 1 reset, 2 center + face down, 3 center, 4 face down |
| `ZoomControl` | continuous zoom: 1 in, 0 stop, −1 out |
| `SetZoom` | absolute zoom |
| `SetVideoMode` | what video1 / video2 carry |
| `GetTemperature` | 0 point, 1 box, 2 full frame |
| `ThermalControl` | 0 palette, 1 gain, 2 shutter (FFC) |

```cpp
#include "unipod_msgs/msg/gimbal_attitude.hpp"
#include "unipod_msgs/srv/set_gimbal_angle.hpp"
```

```python
from unipod_msgs.msg import GimbalAttitude
from unipod_msgs.srv import SetGimbalAngle
```
