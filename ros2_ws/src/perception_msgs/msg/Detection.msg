# A single 2D object detection.

std_msgs/Header header

# Bounding box (image coordinates, pixels)
float64 bbox_x       # top-left x
float64 bbox_y       # top-left y
float64 bbox_width
float64 bbox_height

# Classification
string class_name    # e.g. "person", "car"
int32 class_id       # COCO class id (0 = person, 2 = car, ...)
float32 confidence   # [0.0, 1.0]