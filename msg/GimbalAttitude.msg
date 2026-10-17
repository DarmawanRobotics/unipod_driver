# Gimbal attitude from the pod (0x0D), NED, rotation order yaw -> roll -> pitch.
# Yaw is the magnetic encoder angle relative to the mount, positive to the right;
# pitch positive up (-90 straight down .. 30).
std_msgs/Header header

float32 yaw     # deg
float32 pitch   # deg
float32 roll    # deg

float32 yaw_rate    # deg/s
float32 pitch_rate  # deg/s
float32 roll_rate   # deg/s
