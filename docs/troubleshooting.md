# Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `connected: false` | address, subnet or port | [network.md](network.md); `ping`; try `transport:=udp` |
| connects and drops every few seconds on TCP | heartbeat not answered | try `transport:=udp` |
| service answers `success: false` after ~1 s | no ACK within `request_timeout_ms` (one retry) | raise `request_timeout_ms`; check the link |
| `set_angle` pitch clipped | pitch outside −90..30° | |
| `set_zoom` refused | above `zoom_max` in `status` | |
| `thermal_control` palette refused | value not in the list of `GimbalStatus.msg` | |
| `attitude` silent | `attitude_rate_hz: 0` | 2, 4, 5, 10, 20, 50 or 100 |
| `cannot open rtsp://...` repeating | stream path or decoder | `ffprobe` the URL; `decoder: ffmpeg` |
| image freezes, then `lost ... reconnecting` | stream dropped | automatic; lower `timeout_ms` to notice sooner |
| serial: permission denied | user not in `dialout` | `sudo usermod -aG dialout $USER` |

```bash
ros2 launch unipod_driver unipod_driver.launch.py ip:=<ip> 2>&1 | tee unipod.log
```
