#!/usr/bin/env python3

import torch
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import time

# --- Monkey patch for PyTorch 2.6 weights_only issue ---
orig_load = torch.load
def patched_load(*args, **kwargs):
    kwargs["weights_only"] = False
    return orig_load(*args, **kwargs)
torch.load = patched_load
# -------------------------------------------------------

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.bridge = CvBridge()

        # Load YOLOv8 model
        self.model = YOLO("yolov8n.pt")  # pre-trained COCO model
        self.prev_time = time.time()

        # Subscriber to the image topic
        self.subscription = self.create_subscription(Image, '/carla/hero/rgb_front/image', self.listener_callback, 10)
        self.get_logger().info("Image Subscriber Node Started...")

    def listener_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV BGR image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Pass frame to YOLO detection
            self.yolo(cv_image)

        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

    def yolo(self, image):
        # Run inference
        results = self.model.predict(source=image, save=False, conf=0.5, verbose=False) # 

        # Draw detections
        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = box.xyxy[0]  # Bounding box coordinates
                conf = float(box.conf[0])      # Confidence score
                cls = int(box.cls[0])          # Class ID
                label = self.model.names[cls]

                # Draw bounding box and label
                cv2.rectangle(image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.putText(image, f"{label} {conf:.2f}", (int(x1), int(y1)-5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # Show FPS
        curr_time = time.time()
        fps = 1 / (curr_time - self.prev_time)
        self.prev_time = curr_time
        cv2.putText(image, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

        # Display image
        cv2.imshow("YOLOv8 Detection", image)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt as e:
        node.get_logger().info(f"The exception got is: {e}")
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
