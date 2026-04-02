std_msgs/Header header

uint8 TARGET_PERSON = 0
uint8 TARGET_CAR = 1
uint8 TARGET_BUS = 2
uint8 TARGET_TRUCK = 3
uint8 TARGET_ANY = 255

uint8 TRACKING_NORMAL_AI = 0
uint8 TRACKING_INTERMITTENT = 1
uint8 TRACKING_LOST = 2
uint8 TRACKING_CANCELLED = 3
uint8 TRACKING_NORMAL_OBJECT = 4

uint16 pos_x
uint16 pos_y
uint16 width
uint16 height
uint8 target_id
uint8 tracking_status
