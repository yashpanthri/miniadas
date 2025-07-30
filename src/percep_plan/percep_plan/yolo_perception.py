#!/usr/bin/env python3

import torch.serialization
import torch.nn.modules.container as container
from ultralytics.nn.tasks import DetectionModel
from ultralytics.nn.modules import Conv, C2f, Bottleneck, Detect
from ultralytics.nn.modules import Conv, C2f, Bottleneck, Detect, Concat, SPPF, DFL   # ← add Concat, SPPF, DFL
torch.serialization.add_safe_globals([
    DetectionModel,
    container.Sequential,
    Conv,
    C2f,
    Bottleneck,
    Detect
])

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import time

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')

        torch.serialization.add_safe_globals([
            DetectionModel,
            container.Sequential,
            Conv,
            C2f,
            Bottleneck,
            Detect,
            Concat,      # ← newly whitelisted
            SPPF,        # ← newly whitelisted
            DFL          # ← newly whitelisted
        ])
        # Create CvBridge object
        self.bridge = CvBridge()

        # Subscriber to the image topic
        self.subscription = self.create_subscription(Image, '/carla/hero/rgb_front/image', self.listener_callback, 10)


        # Allow YOLO weights to load safely
        # Allow PyTorch to load YOLOv8 checkpoint safely
        # torch.serialization.add_safe_globals([DetectionModel, container.Sequential, Conv]) # This line is now redundant as it's moved to the top


        # 1. Load the YOLOv8 model (pre-trained on COCO dataset)
        self.model = YOLO("yolov8n.pt")
        self.prev_time = time.time()

        # self.subscription  # prevent unused variable warning
        self.get_logger().info("Image Subscriber Node Started...")

    def listener_callback(self, msg):
        try:
            # Convert ROS Image message (bgra8) to OpenCV BGR image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Display the image using OpenCV
            cv2.imshow("CARLA Front Camera", cv_image)
            self.yolo(cv_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

    def yolo(self, image):

        # 2. Load an image
        # Image already loaded

        # 3. Run inference
        self.results = self.model.predict(source=image, save=False, conf=0.5)

        # 4. Loop through detections and display
        for r in self.results:
            for box in r.boxes:
                x1, y1, x2, y2 = box.xyxy[0]  # Bounding box coordinates
                conf = box.conf[0]             # Confidence score
                cls = int(box.cls[0])          # Class ID
                label = self.model.names[cls]

                # Draw bounding box
                cv2.rectangle(image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.putText(image, f"{label} {conf:.2f}", (int(x1), int(y1)-5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # 5. Show the result
        cv2.imshow("YOLOv8 Detection", image)
        cv2.waitKey(0)

        # Show FPS
        curr_time = time.time()
        fps = 1 / (curr_time - self.prev_time)
        self.prev_time = curr_time
        cv2.putText(image, f"FPS: {fps:.2f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

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
