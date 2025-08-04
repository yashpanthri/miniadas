#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class CarlaImageSaver(Node):
    def __init__(self):
        super().__init__('carla_image_saver')

        # Folder to save images
        self.save_folder = "/home/yashpanthri-unbuntu22/CARLA_PROJECT/mini_adas/src/percep_plan/percep_plan/carla_images"  # <-- Change this path
        os.makedirs(self.save_folder, exist_ok=True) 

        # Frame counter
        self.frame_count = 0
        self.save_interval = 5  # Save every 5th frame

        # ROS <-> OpenCV bridge
        self.bridge = CvBridge()

        # Subscribe to CARLA ego vehicle camera topic
        self.subscription = self.create_subscription(
            Image,
            '/carla/hero/rgb_front/image',
            self.image_callback,
            10
        )
        self.get_logger().info("Carla Image Saver Node Started...")

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Increment frame counter
            self.frame_count += 1

            # Save every 5th frame
            if self.frame_count % self.save_interval == 0:
                file_name = f"frame_{(int)(self.frame_count/self.save_interval)}.png" # (int)self.frame_count // self.save_interval -> gives integer number to each frame
                file_path = os.path.join(self.save_folder, file_name)

                # Save image as PNG
                cv2.imwrite(file_path, cv_image)
                self.get_logger().info(f"Saved: {file_path}")

        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CarlaImageSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down image saver node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
