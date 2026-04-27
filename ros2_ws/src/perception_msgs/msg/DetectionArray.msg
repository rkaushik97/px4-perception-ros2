# Array of detections from a single image frame.

std_msgs/Header header
Detection[] detections

# Performance metadata (this frame only)
float32 inference_latency_ms
float32 image_width
float32 image_height