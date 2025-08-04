#!/usr/bin/env python3
# This tells the system to use Python 3 interpreter to run the script.

import torch                       # PyTorch library for loading the YOLO model weights
import rclpy                       # ROS 2 Python client library
from rclpy.node import Node        # Base class for creating ROS 2 nodes
from sensor_msgs.msg import Image  # ROS message type for images
from cv_bridge import CvBridge     # Converts ROS Image messages to OpenCV images
import cv2                         # OpenCV for image processing and visualization
# from ultralytics import YOLO       # Ultralytics YOLO library for object detection
import time                        # Used for calculating FPS (frames per second)
import numpy as np

import torch.nn as nn
import torchvision.transforms as T
import torch.nn.functional as F
# from torchvision import models

# --- Monkey patch for PyTorch 2.6 weights_only issue ---
# Some YOLO models fail to load in PyTorch 2.6 due to 'weights_only' argument restriction.
# This patch ensures the YOLO model can be loaded properly.

orig_load = torch.load
def patched_load(*args, **kwargs):
    kwargs["weights_only"] = False   # Forces torch.load to ignore the 'weights_only' flag
    return orig_load(*args, **kwargs)
torch.load = patched_load
# -------------------------------------------------------

class ConvBNReLU(nn.Module):
    def __init__(self, in_ch, out_ch, k=3, s=1, p=1):
        super().__init__()
        self.conv = nn.Conv2d(in_ch, out_ch, k, s, p, bias=False)
        self.bn = nn.BatchNorm2d(out_ch)
        self.relu = nn.ReLU(inplace=True)
    def forward(self, x):
        return self.relu(self.bn(self.conv(x)))

class BiSeNetV2(nn.Module):
    def __init__(self, n_classes=2):
        super(BiSeNetV2, self).__init__()
        self.layer1 = ConvBNReLU(3, 64)
        self.layer2 = ConvBNReLU(64, 128)
        self.layer3 = ConvBNReLU(128, 256)
        self.layer4 = ConvBNReLU(256, 128)
        self.layer5 = nn.Conv2d(128, n_classes, 1)

    def forward(self, x):
        x = self.layer1(x)
        x = F.max_pool2d(x, 2)
        x = self.layer2(x)
        x = F.max_pool2d(x, 2)
        x = self.layer3(x)
        x = F.interpolate(self.layer4(x), scale_factor=4, mode='bilinear', align_corners=False)
        return self.layer5(x)

class ImageSubscriber(Node):
    def __init__(self):
        # Initialize ROS 2 node with the name 'image_subscriber'
        super().__init__('image_subscriber')
        self.bridge = CvBridge()  # Create bridge object for ROS-to-OpenCV image conversion

        # Load YOLOv8 pre-trained model (Nano version) for object detection
        # self.model = YOLO("yolov8n.pt")
        # self.model = YOLO("yolov8n-seg.pt")  # Pre-trained segmentation model on COCO
        self.prev_time = time.time()  # Store time of last frame for FPS calculation

        # ----------------------------------------------------------------
        # # Load DeepLabV3 pretrained model (ResNet101 backbone)
        # self.deeplab_model = models.segmentation.deeplabv3_resnet101(pretrained=True)
        # self.deeplab_model.eval()

        # # Define transforms for model input
        # self.transform = T.Compose([
        #     T.ToPILImage(),
        #     T.Resize((520, 520)),
        #     T.ToTensor(),
        #     T.Normalize(mean=[0.485, 0.456, 0.406],
        #                 std=[0.229, 0.224, 0.225])
        # ])
        # ----------------------------------------------------------------
        # BiSeNetV2
        # ----------------------------------------------------------------
        # Force CPU usage to avoid CUDA memory issues
        self.device = 'cpu'  # 'cuda' if torch.cuda.is_available() else 'cpu'
        self.bisenet_model = BiSeNetV2(n_classes=2).to(self.device)
        
        # Try to load pre-trained weights if available
        try:
            self.bisenet_model.load_state_dict(torch.load("bisenetv2_carla.pth", map_location=self.device))
            self.get_logger().info("Loaded pre-trained BiSeNetV2 weights")
        except FileNotFoundError:
            self.get_logger().warning("No pre-trained weights found. Using random initialization.")
        
        self.bisenet_model.eval()
        self.bisenet_transform = T.Compose([
            T.ToPILImage(),
            T.Resize((256, 256)),
            T.ToTensor()
        ])
        # ----------------------------------------------------------------

        # Subscribe to the CARLA simulator's camera image topic
        # '/carla/hero/rgb_front/image' publishes front camera images
        self.subscription = self.create_subscription(
            Image,                   # Message type
            '/carla/hero/rgb_front/image',  # Topic name
            self.camera_cb,  # Callback function when new image is received
            10                       # Queue size
        )
        self.get_logger().info("Image Subscriber Node Started with BiSeNetV2 Road Segmentation...")

    def camera_cb(self, msg):
        # This function is triggered every time a new image is published on the topic
        try:
            # Convert ROS Image message to OpenCV BGR image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Display the original image
            cv2.imshow("CARLA Front Camera", cv_image)

            # # Run YOLO detection on the converted image
            # self.yolo(cv_image)

            # # Passing frame to YOLO segmentation
            # self.yolo_segmentation(cv_image)
            self.bisenet_segmentation(cv_image)

            # self.deeplab_segmentation(cv_image)

        except Exception as e:
            # Logs an error if image conversion fails
            self.get_logger().error(f"Failed to convert image: {e}")

    # def yolo(self, image):
    #     # Run YOLO inference on the current frame
    #     results = self.model.predict(
    #         source=image,   # Input frame
    #         save=False,     # Do not save detection images to disk
    #         conf=0.5,       # Confidence threshold (only detections above 50% confidence are considered)
    #         verbose=False   # Disable extra console logs
    #     )

    #     # Loop through the results for each detection
    #     for r in results:
    #         for box in r.boxes:
    #             # Extract bounding box coordinates
    #             x1, y1, x2, y2 = box.xyxy[0]
    #             # Extract confidence score of detection
    #             conf = float(box.conf[0])
    #             # self.get_logger().info(f"CLS: {box.conf}")

    #             # Extract detected object's class ID
    #             cls = int(box.cls[0])
    #             # self.get_logger().info(f"CLS: {box.cls}")
    #             # Get class name from model labels
    #             label = self.model.names[cls]

    #             # Draw rectangle around detected object
    #             cv2.rectangle(image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
    #             # Draw label and confidence score above the bounding box
    #             cv2.putText(image, f"{label} {conf:.2f}", (int(x1), int(y1)-5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    #     # Calculate FPS
    #     curr_time = time.time()
    #     fps = 1 / (curr_time - self.prev_time)
    #     self.prev_time = curr_time

    #     # Display FPS on the image
    #     cv2.putText(image, f"FPS: {fps:.2f}", (10, 30),
    #                 cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

    #     # Show the processed image with detections in a window
    #     cv2.imshow("YOLOv8 Detection", image)
    #     cv2.waitKey(1)  # Wait briefly to update window

    # def yolo_segmentation(self, image):
    #     # Run segmentation inference
    #     results = self.model.predict(source=image, save=False, conf=0.5, verbose=False, iou = 0.5)
    #     for r in results:
    #         # Get masks and boxes
    #         if r.masks is not None:
    #             masks = r.masks.data.cpu().numpy()
    #             boxes = r.boxes

    #             for mask, box in zip(masks, boxes):
    #                 x1, y1, x2, y2 = box.xyxy[0]
    #                 conf = float(box.conf[0])
    #                 cls = int(box.cls[0])
    #                 label = self.model.names[cls]

    #                 # Draw bounding box
    #                 cv2.rectangle(image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
    #                 cv2.putText(image, f"{label} {conf:.2f}", (int(x1), int(y1)-5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    #                 # Resize mask to image size
    #                 mask_resized = cv2.resize(mask, (image.shape[1], image.shape[0]))
    #                 mask_binary = (mask_resized > 0.5).astype(np.uint8) * 255 # Converts YOLO's soft mask (0-1 probability per pixel) into a binary mask image: first threshold >0.5 to mark pixels likely belonging to the object (True=object, False=background), then convert boolean values to 8-bit integers (1 for object, 0 for background), and finally multiply by 255 to get a standard black-and-white mask (255=object region, 0=background) suitable for OpenCV display/processing.

    #                 # Create colored mask overlay
    #                 colored_mask = np.zeros_like(image, dtype=np.uint8)
    #                 color = (0, 0, 255)  # Red mask
    #                 colored_mask[:, :, 0] = mask_binary * (color[0] / 255)
    #                 colored_mask[:, :, 1] = mask_binary * (color[1] / 255)
    #                 colored_mask[:, :, 2] = mask_binary * (color[2] / 255)

    #                 # Blend mask with image
    #                 image = cv2.addWeighted(image, 1, colored_mask, 0.5, 0)

    #     # Show FPS
    #     curr_time = time.time()
    #     fps = 1 / (curr_time - self.prev_time)
    #     self.prev_time = curr_time
    #     cv2.putText(image, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

    #     # Display segmented image
    #     cv2.imshow("YOLOv8 Semantic Segmentation", image)
    #     cv2.waitKey(1)

    # # -------------------- DeepLabV3 Segmentation ----------------------
    # def deeplab_segmentation(self, image):
    #     # Convert BGR to RGB
    #     img_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    #     input_tensor = self.transform(img_rgb).unsqueeze(0)

    #     with torch.no_grad():
    #         output = self.deeplab_model(input_tensor)['out'][0]
    #     output_predictions = output.argmax(0).byte().cpu().numpy()

    #     # Generate road mask (PascalVOC class "road" often = 15)
    #     road_class = 15
    #     road_mask = (output_predictions == road_class).astype(np.uint8) * 255

    #     # Resize mask to original image size
    #     road_mask = cv2.resize(road_mask, (image.shape[1], image.shape[0]))

    #     # Overlay mask on image
    #     colored_mask = np.zeros_like(image)
    #     colored_mask[:, :, 1] = road_mask  # Green color for road
    #     blended = cv2.addWeighted(image, 1.0, colored_mask, 0.5, 0)

    #     # FPS calculation
    #     curr_time = time.time()
    #     fps = 1 / (curr_time - self.prev_time)
    #     self.prev_time = curr_time

    #     cv2.putText(blended, f"FPS: {fps:.2f}", (10, 30),
    #                 cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

    #     # Display
    #     cv2.imshow("DeepLabV3 Road Segmentation", blended)
    #     cv2.waitKey(1)

    # -------------------- BiSeNetV2 ----------------------
    def bisenet_segmentation(self, image):
        img_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        img_t = self.bisenet_transform(img_rgb).unsqueeze(0).to(self.device)

        with torch.no_grad():
            pred = self.bisenet_model(img_t)
            mask = torch.argmax(pred, dim=1).squeeze(0).cpu().numpy()

        # Convert mask to overlay
        mask = cv2.resize(mask, (image.shape[1], image.shape[0]))
        mask_colored = np.zeros_like(image)
        mask_colored[:, :, 1] = (mask * 255).astype('uint8')  # Green road area
        blended = cv2.addWeighted(image, 1, mask_colored, 0.5, 0)

        # FPS calculation
        curr_time = time.time()
        fps = 1 / (curr_time - self.prev_time)
        self.prev_time = curr_time
        cv2.putText(blended, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

        cv2.imshow("BiSeNetV2 Road Segmentation", blended)
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
