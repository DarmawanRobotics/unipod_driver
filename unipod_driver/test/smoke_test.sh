#!/usr/bin/env bash
# End-to-end smoke test: the launch file against test/fake_pod.py over UDP on the loopback.
# Topics, every service, ptz_cmd and its timeout, and both camera nodes staying up without a
# stream. Needs a sourced workspace.
set -u
# ros2 CLI output into a pipe is block-buffered and lost when timeout kills it
export PYTHONUNBUFFERED=1

here="$(cd "$(dirname "$0")" && pwd)"
work="$(mktemp -d)"
port=37260
fail=0
pids=()

cleanup() {
  for p in "${pids[@]}"; do kill -INT "$p" 2>/dev/null; done
  sleep 1
  for p in "${pids[@]}"; do kill -9 "$p" 2>/dev/null; done
}
trap cleanup EXIT

check() {  # check <name> <pattern> <command...>
  local name="$1" pattern="$2"
  shift 2
  local out
  out="$(timeout 20 "$@" 2>&1)"
  if grep -qE "$pattern" <<<"$out"; then
    echo "ok   $name"
  else
    echo "FAIL $name"
    echo "$out" | tail -n 20
    fail=1
  fi
}

python3 "$here/fake_pod.py" --port "$port" --log "$work/pod.log" &
pids+=($!)
ros2 launch unipod_driver unipod_driver.launch.py transport:=udp ip:=127.0.0.1 \
  > "$work/launch.log" 2>&1 &
pids+=($!)
sleep 8

# Foxy's `ros2 topic echo` has no --once: listen for a few seconds instead, with the type
# given so it does not depend on discovery being finished
check "status connected" "connected: true" \
  timeout 8 ros2 topic echo /unipod/status unipod_msgs/msg/GimbalStatus
check "attitude stream" "yaw:" \
  timeout 8 ros2 topic echo /unipod/attitude unipod_msgs/msg/GimbalAttitude
check "temperature stream" "temp_max: 60" \
  timeout 8 ros2 topic echo /unipod/thermal/temperature unipod_msgs/msg/ThermalData
check "set_zoom" "success=True" \
  ros2 service call /unipod/set_zoom unipod_msgs/srv/SetZoom "{zoom: 7.5}"
check "zoom" "success=True, zoom=2.5" \
  ros2 service call /unipod/zoom unipod_msgs/srv/ZoomControl "{command: 1}"
check "set_angle" "success=True, yaw=30.0, pitch=-20.0" \
  ros2 service call /unipod/set_angle unipod_msgs/srv/SetGimbalAngle "{yaw: 30.0, pitch: -20.0}"
check "recenter" "success=True" \
  ros2 service call /unipod/recenter unipod_msgs/srv/Recenter "{mode: 3}"
check "set_video_mode" "success=True, main_stream=2, sub_stream=0" \
  ros2 service call /unipod/set_video_mode unipod_msgs/srv/SetVideoMode \
  "{main_stream: 2, sub_stream: 0}"
check "get_temperature point" "temp_max=36.5" \
  ros2 service call /unipod/get_temperature unipod_msgs/srv/GetTemperature \
  "{mode: 0, x: 320, y: 256}"
check "get_temperature region" "temp_max=45\.(1|09)" \
  ros2 service call /unipod/get_temperature unipod_msgs/srv/GetTemperature \
  "{mode: 1, x: 10, y: 10, x2: 100, y2: 100}"
check "thermal palette" "success=True, value=3" \
  ros2 service call /unipod/thermal_control unipod_msgs/srv/ThermalControl \
  "{command: 0, value: 3}"
check "thermal bad palette" "success=False" \
  ros2 service call /unipod/thermal_control unipod_msgs/srv/ThermalControl \
  "{command: 0, value: 1}"
check "shutter" "success=True" \
  ros2 service call /unipod/thermal_control unipod_msgs/srv/ThermalControl "{command: 2}"

timeout 10 ros2 topic pub --times 5 -r 10 /unipod/ptz_cmd geometry_msgs/msg/Twist \
  "{angular: {z: 0.5, y: -0.2}}" > /dev/null 2>&1
sleep 1.5  # ptz_cmd_timeout 0.5 s
check "ptz_cmd rotates" "cmd 0x07 32ec" cat "$work/pod.log"   # yaw 50, pitch -20
check "ptz_cmd timeout stops" "cmd 0x07 0000" cat "$work/pod.log"
check "yaml applied (video mode on connect)" "cmd 0x11 0002" cat "$work/pod.log"
check "cameras up, retrying" "cannot open rtsp://127.0.0.1:8554/video1" cat "$work/launch.log"
check "sub camera up" "cannot open rtsp://127.0.0.1:8554/video2" cat "$work/launch.log"

if grep -q "process has died" "$work/launch.log"; then
  echo "FAIL a node died"
  fail=1
fi
if [ "$fail" != 0 ]; then
  echo "---- launch log"
  tail -n 60 "$work/launch.log"
fi
exit "$fail"
