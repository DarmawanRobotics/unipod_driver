# First run on the pod

The driver is tested against a simulated pod
([fake_pod.py](../unipod_driver/test/fake_pod.py)). Fill in the **Result** column on the real
pod; it decides the defaults that go into `unipod_config.yaml`.

```bash
ros2 launch unipod_driver unipod_driver.launch.py ip:=<pod ip>
```

| # | Check | Command | Expected | Result |
|---|---|---|---|---|
| 1 | Link | `ros2 topic echo /unipod/status unipod_msgs/msg/GimbalStatus` | `connected: true`, firmware versions | |
| 2 | Attitude | `ros2 topic hz /unipod/attitude` | `attitude_rate_hz` (10 Hz) | |
| 3 | Directions | `ros2 service call /unipod/set_angle unipod_msgs/srv/SetGimbalAngle "{yaw: 30.0, pitch: -20.0}"` | turns right and down; answer near 30 / −20 | |
| 4 | Manual | `ros2 topic pub -r 10 /unipod/ptz_cmd geometry_msgs/msg/Twist "{angular: {z: 0.3}}"` then stop | turns right, stops 0.5 s after the last message | |
| 5 | Zoom | `ros2 service call /unipod/set_zoom unipod_msgs/srv/SetZoom "{zoom: 10.0}"` | `zoom: 10.0` | |
| 6 | Streams | node log at connect | codec and resolution of video1 / video2 | |
| 7 | Images | `ros2 topic hz /unipod/main/image_raw`, `/unipod/sub/image_raw` | frames at the stream rate | |
| 8 | Video mode | `ros2 service call /unipod/set_video_mode unipod_msgs/srv/SetVideoMode "{main_stream: 2, sub_stream: 0}"` | video1 thermal, video2 zoom | |
| 9 | Temperature | `ros2 service call /unipod/get_temperature unipod_msgs/srv/GetTemperature "{mode: 0, x: 320, y: 256}"` on a hand | 30–36 °C | |
| 10 | Thermal | `ros2 service call /unipod/thermal_control unipod_msgs/srv/ThermalControl "{command: 2}"` | shutter clicks (FFC) | |

## If a check fails

- **1:** see [network.md](network.md); try `transport:=udp`.
- **3 / 4:** directions reversed: `invert_yaw` / `invert_pitch` (ptz_cmd only).
- **7 with Jetson:** set `codec` in `unipod_config.yaml` to what check 6 logged, for the hardware
  decoder.

See also [troubleshooting.md](troubleshooting.md).
