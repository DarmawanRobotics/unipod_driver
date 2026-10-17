# Pod state, polled at status_rate_hz.
std_msgs/Header header

# Video streams (main_stream / sub_stream)
uint8 STREAM_ZOOM = 0
uint8 STREAM_WIDE = 1
uint8 STREAM_THERMAL = 2
uint8 STREAM_ZOOM_THERMAL = 3   # zoom stitched with thermal
uint8 STREAM_WIDE_THERMAL = 4
uint8 STREAM_ZOOM_WIDE = 5
uint8 STREAM_NONE = 6

# Thermal palettes
uint8 PALETTE_WHITE_HOT = 0
uint8 PALETTE_SEPIA = 2
uint8 PALETTE_IRONBOW = 3
uint8 PALETTE_RAINBOW = 4
uint8 PALETTE_NIGHT = 5
uint8 PALETTE_AURORA = 6
uint8 PALETTE_RED_HOT = 7
uint8 PALETTE_JUNGLE = 8
uint8 PALETTE_MEDICAL = 9
uint8 PALETTE_BLACK_HOT = 10
uint8 PALETTE_GLORY_HOT = 11

bool connected          # a frame from the pod within link_timeout

float32 zoom            # current hybrid zoom, x
float32 zoom_max        # x

uint8 main_stream       # rtsp://<ip>:8554/video1
uint8 sub_stream        # rtsp://<ip>:8554/video2

uint8 thermal_palette
uint8 thermal_gain      # 0 low, 1 high

string camera_firmware
string gimbal_firmware
