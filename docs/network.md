# Network setup

| | |
|---|---|
| factory IP | `192.168.144.25` |
| control | TCP / UDP `37260`, or TTL serial `/dev/ttyUSB0` 115200 8N1 |
| video | `rtsp://<ip>:8554/video1` (main), `/video2` (sub) |

`unipod_driver/config/unipod_config.yaml` and the launch default to `10.21.31.26`; set `ip` to
the pod's actual address.

## An address in the pod's subnet

Quick test, lost on reboot:

```bash
sudo ip addr add 192.168.144.100/24 dev eth0      # your interface: ip -br link
ping -c 3 192.168.144.25
```

Permanent, with netplan (`/etc/netplan/60-unipod.yaml`):

```yaml
network:
  version: 2
  ethernets:
    eth0:
      addresses:
        - 192.168.144.100/24
```

```bash
sudo netplan apply
```

## Check before launching

```bash
ping -c 3 <pod ip>
ffprobe -v error -show_streams rtsp://<pod ip>:8554/video1 | grep -E 'codec_name|width|height'
```

## Serial instead of Ethernet

```bash
sudo usermod -aG dialout $USER     # log out and in once
ros2 launch unipod_driver unipod_driver.launch.py transport:=serial
```
