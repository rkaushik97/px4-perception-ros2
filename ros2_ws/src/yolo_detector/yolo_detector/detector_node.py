#!/usr/bin/env python3
"""YOLOv8 object detection ROS2 node.

Subscribes to a camera image topic, runs YOLOv8 inference on every frame, and
publishes:
  - /yolo/detections        : perception_msgs/DetectionArray (custom message)
  - /yolo/annotated_image   : sensor_msgs/Image with bounding boxes drawn

Inference latency and FPS are tracked over a rolling window and logged every
30 frames for the performance analysis deliverable.
"""

import time
from collections import deque

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from ultralytics import YOLO

from perception_msgs.msg import Detection, DetectionArray


class YoloDetectorNode(Node):
    """ROS2 node that runs YOLOv8 on incoming camera frames.

    Parameters
    ----------
    model_path : str
        Path to a YOLOv8 weights file (.pt). Defaults to ``yolov8n.pt`` which
        ultralytics will auto-download into the current working directory.
    image_topic : str
        ROS2 topic to subscribe to for input images. Defaults to the
        ros_gz_image_bridge topic for the x500_mono_cam airframe.
    confidence_threshold : float
        Minimum detection confidence in [0, 1]. Defaults to 0.50.
    device : str
        Torch device string passed to ultralytics, e.g. ``cuda:0`` or ``cpu``.
    """

    def __init__(self):
        super().__init__("yolo_detector")

        self.declare_parameter("model_path", "yolov8n.pt")
        self.declare_parameter(
            "image_topic",
            "/world/default/model/x500_mono_cam_0/link/camera_link/sensor/camera/image",
        )
        self.declare_parameter("confidence_threshold", 0.50)
        self.declare_parameter("device", "cuda:0")

        model_path = self.get_parameter("model_path").value
        image_topic = self.get_parameter("image_topic").value
        self.conf_threshold = self.get_parameter("confidence_threshold").value
        device = self.get_parameter("device").value

        self.get_logger().info(f"Loading YOLO model: {model_path} on {device}")
        self.model = YOLO(model_path)
        self.model.to(device)
        self.get_logger().info("Model loaded successfully")

        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, 10
        )
        self.detections_pub = self.create_publisher(
            DetectionArray, "/yolo/detections", 10
        )
        self.annotated_pub = self.create_publisher(Image, "/yolo/annotated_image", 10)

        self.latencies = deque(maxlen=100)
        self.frame_count = 0
        self.last_log_time = time.time()

        self.get_logger().info(f"Subscribed to: {image_topic}")
        self.get_logger().info("YOLO detector node ready")

    def image_callback(self, msg: Image):
        """Run YOLO inference on one image and publish detections + annotated frame."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge conversion failed: {e}")
            return

        t0 = time.perf_counter()
        results = self.model(frame, conf=self.conf_threshold, verbose=False)
        latency_ms = (time.perf_counter() - t0) * 1000.0
        self.latencies.append(latency_ms)
        self.frame_count += 1

        det_array = DetectionArray()
        det_array.header = msg.header
        det_array.image_width = float(frame.shape[1])
        det_array.image_height = float(frame.shape[0])

        result = results[0]
        if result.boxes is not None:
            for box in result.boxes:
                xyxy = box.xyxy[0].cpu().numpy()
                cls_id = int(box.cls[0].cpu().numpy())
                conf = float(box.conf[0].cpu().numpy())

                detection = Detection()
                detection.header = msg.header
                detection.bbox_x = float(xyxy[0])
                detection.bbox_y = float(xyxy[1])
                detection.bbox_width = float(xyxy[2] - xyxy[0])
                detection.bbox_height = float(xyxy[3] - xyxy[1])
                detection.class_name = self.model.names[cls_id]
                detection.class_id = cls_id
                detection.confidence = conf

                det_array.detections.append(detection)

        det_array.inference_latency_ms = float(latency_ms)
        self.detections_pub.publish(det_array)

        annotated = result.plot()
        annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        annotated_msg.header = msg.header
        self.annotated_pub.publish(annotated_msg)

        if self.frame_count % 30 == 0:
            mean_latency = float(np.mean(self.latencies))
            fps = 1000.0 / mean_latency if mean_latency > 0 else 0.0
            self.get_logger().info(
                f"Frames: {self.frame_count} | "
                f"Latency: {mean_latency:.1f} ms | "
                f"FPS: {fps:.1f} | "
                f"Detections this frame: {len(det_array.detections)}"
            )


def main(args=None):
    """Entry point for `ros2 run yolo_detector detector_node`."""
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
