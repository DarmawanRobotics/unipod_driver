# Temperatures from the thermal camera, degrees Celsius. Pixel coordinates are in the thermal
# image (640x512, or 1280x1024 with AI super-resolution).
std_msgs/Header header

uint8 POINT = 0
uint8 REGION = 1
uint8 FRAME = 2
uint8 mode

# POINT: the measured point (x1, y1). REGION: the box the camera actually measured (it scales
# with the thermal digital zoom). FRAME: unused.
uint16 x1
uint16 y1
uint16 x2
uint16 y2

float32 temp_max
float32 temp_min    # POINT: same as temp_max
uint16 max_x
uint16 max_y
uint16 min_x
uint16 min_y
