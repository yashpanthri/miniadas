#!/usr/bin/env python3
"""
BiSeNetV2 Road Segmentation Perception Node
This script uses a trained BiSeNetV2 model specifically for road segmentation.
BiSeNetV2 is a lightweight, real-time semantic segmentation model designed for
efficient pixel-level classification of road areas.
"""

import rclpy                       # ROS 2 Python client library for node creation
from rclpy.node import Node        # Base class for creating ROS 2 nodes
from sensor_msgs.msg import Image  # ROS message type for camera images
from cv_bridge import CvBridge     # Converts ROS Image messages to OpenCV images
import cv2                         # OpenCV for image processing and visualization
import numpy as np                 # Numerical computing library for array operations
import time                        # Time library for FPS calculation
import torch                       # PyTorch deep learning framework
import torchvision.transforms as T # PyTorch image transformations
from PIL import Image as PILImage  # PIL for image processing

# Import BiSeNetV2 model from your existing file
from yolo_perception import BiSeNetV2

class BiSeNetV2RoadNode(Node):
    """
    ROS 2 Node for road segmentation using BiSeNetV2 deep learning model.
    This node subscribes to camera images and performs semantic segmentation
    to identify road areas at the pixel level.
    """
    
    def __init__(self):
        """Initialize the BiSeNetV2RoadNode with model loading and ROS 2 setup."""
        # Initialize ROS 2 node with the name 'bisenetv2_road_node'
        super().__init__('bisenetv2_road_node')
        
        # Create bridge object for converting ROS images to OpenCV format
        self.bridge = CvBridge()
        
        # Store the previous time for FPS calculation
        self.prev_time = time.time()

        # Device configuration - force CPU to avoid CUDA memory issues
        self.device = 'cpu'  # Force CPU to avoid CUDA issues
        
        # Load BiSeNetV2 model for road segmentation
        try:
            # Initialize BiSeNetV2 model with 1 class (binary segmentation: road vs non-road)
            self.model = BiSeNetV2(n_classes=1).to(self.device)
            
            # Try to load pre-trained model weights
            try:
                # Load the best trained model weights from file
                self.model.load_state_dict(torch.load("bisenetv2_carla_best.pth", map_location=self.device))
                self.get_logger().info("Loaded trained BiSeNetV2 model for road segmentation")
            except FileNotFoundError:
                # If trained model not found, use untrained model (random weights)
                self.get_logger().warning("Trained model not found. Using untrained BiSeNetV2 model")
            
            # Set model to evaluation mode (disables dropout and batch normalization updates)
            self.model.eval()
            
        except Exception as e:
            # Log any errors that occur during model loading
            self.get_logger().error(f"Failed to load BiSeNetV2 model: {e}")
            return

        # Define image preprocessing transforms for the model
        # These transforms prepare the input image for the neural network
        self.transform = T.Compose([
            T.ToPILImage(),                                    # Convert numpy array to PIL Image
            T.Resize((256, 256)),                             # Resize to model input size
            T.ToTensor(),                                     # Convert to PyTorch tensor
            T.Normalize(mean=[0.485, 0.456, 0.406],          # ImageNet normalization
                       std=[0.229, 0.224, 0.225])            # Standard deviation values
        ])

        # Subscribe to the CARLA simulator's camera image topic
        # '/carla/hero/rgb_front/image' publishes front camera images from the ego vehicle
        self.subscription = self.create_subscription(
            Image,                   # Message type: ROS Image
            '/carla/hero/rgb_front/image',  # Topic name: CARLA front camera
            self.camera_cb,          # Callback function when new image is received
            10                       # Queue size: number of messages to buffer
        )
        
        # Log that the node has started successfully
        self.get_logger().info("BiSeNetV2 Road Segmentation Node Started...")

    def camera_cb(self, msg):
        """
        Camera callback function - triggered when a new image is published.
        
        Args:
            msg: ROS Image message containing the camera frame
        """
        try:
            # Convert ROS Image message to OpenCV BGR image format
            # 'bgr8' encoding is standard for OpenCV operations
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Display the original camera image in a window
            cv2.imshow("CARLA Front Camera", cv_image)

            # Run BiSeNetV2 road segmentation on the image
            self.bisenetv2_road_segmentation(cv_image)

        except Exception as e:
            # Log any errors that occur during image processing
            self.get_logger().error(f"Failed to process image: {e}")

    def bisenetv2_road_segmentation(self, image):
        """
        Run BiSeNetV2 road segmentation inference on the input image.
        
        Args:
            image: OpenCV BGR image to process for road segmentation
        """
        
        try:
            # Preprocess image for the neural network
            # Convert BGR to RGB (PyTorch expects RGB format)
            image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            
            # Apply transforms and add batch dimension for model input
            input_tensor = self.transform(image_rgb).unsqueeze(0).to(self.device)
            
            # Run inference with gradient computation disabled for efficiency
            with torch.no_grad():
                # Forward pass through the BiSeNetV2 model
                output = self.model(input_tensor)
                
                # Apply sigmoid activation for binary segmentation
                # Sigmoid converts logits to probabilities between 0 and 1
                if output.shape[1] == 1:  # Single channel output (binary segmentation)
                    prob_map = torch.sigmoid(output)
                else:  # Multi-channel output, take first channel for binary case
                    prob_map = torch.sigmoid(output[:, 0:1, :, :])
                
                # Convert PyTorch tensor to numpy array and move to CPU
                prob_map = prob_map.squeeze().cpu().numpy()
                
                # Resize probability map to original image size for visualization
                prob_map_resized = cv2.resize(prob_map, (image.shape[1], image.shape[0]))
                
                # Create binary mask by thresholding probability map
                # Pixels with probability > 0.5 are classified as road
                binary_mask = (prob_map_resized > 0.5).astype(np.uint8) * 255
                
                # Clean up binary mask with morphological operations
                # Create 5x5 kernel for morphological operations
                kernel = np.ones((5, 5), np.uint8)
                # Close operation: fills small holes in road regions
                binary_mask = cv2.morphologyEx(binary_mask, cv2.MORPH_CLOSE, kernel)
                # Open operation: removes small noise from the mask
                binary_mask = cv2.morphologyEx(binary_mask, cv2.MORPH_OPEN, kernel)
                
                # Create and display the final visualization with road overlay
                self.create_road_visualization(image, binary_mask, prob_map_resized)
                
        except Exception as e:
            # Log any errors that occur during inference
            self.get_logger().error(f"BiSeNetV2 inference failed: {e}")
            # Show original image if inference fails
            cv2.imshow("BiSeNetV2 Road Segmentation", image)

    def create_road_visualization(self, image, binary_mask, prob_map):
        """
        Create and display visualization of road segmentation results.
        
        Args:
            image: Original camera image
            binary_mask: Binary road segmentation mask (0 or 255)
            prob_map: Probability map from model output (0.0 to 1.0)
        """
        
        # Create colored overlay mask (same size as original image)
        colored_mask = np.zeros_like(image)
        # Set green channel to binary mask (green = road areas)
        colored_mask[:, :, 1] = binary_mask
        
        # Blend original image with colored mask for visualization
        # 70% original image + 30% green road overlay
        overlay = cv2.addWeighted(image, 0.7, colored_mask, 0.3, 0)
        
        # Calculate road percentage coverage
        # Count white pixels in binary mask divided by total pixels
        road_percentage = np.sum(binary_mask > 0) / (image.shape[0] * image.shape[1]) * 100
        
        # Add road percentage text to the overlay
        cv2.putText(overlay, f"Road: {road_percentage:.1f}%", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        
        # Calculate and display FPS (frames per second)
        curr_time = time.time()  # Get current time
        fps = 1 / (curr_time - self.prev_time)  # Calculate FPS
        self.prev_time = curr_time  # Update previous time for next frame
        
        # Add FPS text to the overlay
        cv2.putText(overlay, f"FPS: {fps:.2f}", (10, 70),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
        
        # Display the final result in a window
        cv2.imshow("BiSeNetV2 Road Segmentation", overlay)
        # Wait briefly to update the window (1ms)
        cv2.waitKey(1)

def main(args=None):
    """
    Main function to initialize and run the ROS 2 BiSeNetV2 road segmentation node.
    
    Args:
        args: Command line arguments (optional)
    """
    # Initialize ROS 2 communication
    rclpy.init(args=args)
    
    # Create the BiSeNetV2 road segmentation node
    node = BiSeNetV2RoadNode()
    
    try:
        # Keep the node running to continuously process incoming images
        rclpy.spin(node)
    except KeyboardInterrupt as e:
        # Handle graceful shutdown on Ctrl+C
        node.get_logger().info(f"The exception got is: {e}")
    finally:
        # Clean up resources
        node.destroy_node()  # Destroy the node
        cv2.destroyAllWindows()  # Close all OpenCV windows
        rclpy.shutdown()  # Shutdown ROS 2

# Entry point of the script
if __name__ == '__main__':
    main() 