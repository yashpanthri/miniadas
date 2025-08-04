#!/usr/bin/env python3
"""
Road-Only Perception Node
This script focuses specifically on road detection using traditional computer vision techniques:
1. Color-based road detection - Detects road colors (gray, white, brown)
2. Edge-based road detection - Uses edge patterns to identify road boundaries
3. Region-based road detection - Uses geometric trapezoidal regions for road perspective
4. Combined approach - Merges all methods for robust road detection
"""

import rclpy                       # ROS 2 Python client library for node creation
from rclpy.node import Node        # Base class for creating ROS 2 nodes
from sensor_msgs.msg import Image  # ROS message type for camera images
from cv_bridge import CvBridge     # Converts ROS Image messages to OpenCV images
import cv2                         # OpenCV for image processing and computer vision
import numpy as np                 # Numerical computing library for array operations
import time                        # Time library for FPS calculation

class RoadOnlyNode(Node):
    """
    ROS 2 Node for road detection using traditional computer vision techniques.
    This node subscribes to camera images and performs multi-method road detection.
    """
    
    def __init__(self):
        """Initialize the RoadOnlyNode with ROS 2 setup and road detection methods."""
        # Initialize ROS 2 node with the name 'road_only_node'
        super().__init__('road_only_node')
        
        # Create bridge object for converting ROS images to OpenCV format
        self.bridge = CvBridge()
        
        # Store the previous time for FPS calculation
        self.prev_time = time.time()

        # Subscribe to the CARLA simulator's camera image topic
        # '/carla/hero/rgb_front/image' publishes front camera images from the ego vehicle
        self.subscription = self.create_subscription(
            Image,                   # Message type: ROS Image
            '/carla/hero/rgb_front/image',  # Topic name: CARLA front camera
            self.camera_cb,          # Callback function when new image is received
            10                       # Queue size: number of messages to buffer
        )
        
        # Log that the node has started successfully
        self.get_logger().info("Road-Only Perception Node Started...")

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

            # Run the multi-method road detection on the image
            self.road_detection(cv_image)

        except Exception as e:
            # Log any errors that occur during image processing
            self.get_logger().error(f"Failed to convert image: {e}")

    def road_detection(self, image):
        """
        Main road detection function that combines multiple detection methods.
        
        Args:
            image: OpenCV BGR image to process for road detection
        """
        
        # Method 1: Color-based road detection - detects road-like colors
        road_mask_color = self.color_based_road_detection(image)
        
        # Method 2: Edge-based road detection - uses edge patterns
        road_mask_edge = self.edge_based_road_detection(image)
        
        # Method 3: Region-based road detection - uses geometric regions
        road_mask_region = self.region_based_road_detection(image)
        
        # Combine all three detection methods into a single robust mask
        combined_mask = self.combine_road_masks(road_mask_color, road_mask_edge, road_mask_region)
        
        # Create and display the final visualization with road overlay
        self.create_road_visualization(image, combined_mask)

    def color_based_road_detection(self, image):
        """
        Detect road areas based on color characteristics using HSV color space.
        
        Args:
            image: OpenCV BGR image to process
            
        Returns:
            road_mask: Binary mask where white pixels represent detected road areas
        """
        
        # Convert BGR image to HSV color space for better color segmentation
        # HSV separates hue (color), saturation (intensity), and value (brightness)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Initialize list to store different color range masks
        masks = []
        
        # Range 1: Light gray/white colors typical of asphalt/concrete roads
        # Lower bound: low saturation, high value (bright gray/white)
        lower_gray1 = np.array([0, 0, 100])
        # Upper bound: any hue, low saturation, high value
        upper_gray1 = np.array([180, 30, 255])
        # Create binary mask for this color range
        mask1 = cv2.inRange(hsv, lower_gray1, upper_gray1)
        masks.append(mask1)
        
        # Range 2: Dark gray colors typical of asphalt roads
        # Lower bound: low saturation, medium value (dark gray)
        lower_gray2 = np.array([0, 0, 50])
        # Upper bound: any hue, medium saturation, medium value
        upper_gray2 = np.array([180, 50, 150])
        # Create binary mask for this color range
        mask2 = cv2.inRange(hsv, lower_gray2, upper_gray2)
        masks.append(mask2)
        
        # Range 3: Brownish colors typical of dirt roads
        # Lower bound: brown hue (10-20), medium saturation and value
        lower_brown = np.array([10, 50, 50])
        # Upper bound: brown hue, high saturation and value
        upper_brown = np.array([20, 255, 255])
        # Create binary mask for this color range
        mask3 = cv2.inRange(hsv, lower_brown, upper_brown)
        masks.append(mask3)
        
        # Combine all color masks using bitwise OR operation
        # This creates a single mask containing all detected road colors
        road_mask = np.zeros_like(mask1)  # Initialize with zeros
        for mask in masks:
            road_mask = cv2.bitwise_or(road_mask, mask)  # Combine each mask
        
        # Morphological operations to clean up the mask
        # Create a 7x7 kernel for morphological operations
        kernel = np.ones((7, 7), np.uint8)
        # Close operation: fills small holes in the mask
        road_mask = cv2.morphologyEx(road_mask, cv2.MORPH_CLOSE, kernel)
        # Open operation: removes small noise from the mask
        road_mask = cv2.morphologyEx(road_mask, cv2.MORPH_OPEN, kernel)
        
        return road_mask

    def edge_based_road_detection(self, image):
        """
        Detect road areas based on edge patterns and geometric constraints.
        
        Args:
            image: OpenCV BGR image to process
            
        Returns:
            road_mask: Binary mask where white pixels represent detected road edges
        """
        
        # Convert BGR image to grayscale for edge detection
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Apply bilateral filter to preserve edges while smoothing noise
        # This filter is edge-preserving and reduces noise without blurring edges
        blurred = cv2.bilateralFilter(gray, 9, 75, 75)
        
        # Detect edges with different sensitivity thresholds
        # Lower threshold (30, 100): detects more edges including weak ones
        edges1 = cv2.Canny(blurred, 30, 100)
        # Higher threshold (50, 150): detects only strong edges
        edges2 = cv2.Canny(blurred, 50, 150)
        
        # Combine both edge detections using bitwise OR
        # This captures both strong and weak road edges
        edges = cv2.bitwise_or(edges1, edges2)
        
        # Create a mask for the lower part of the image where roads typically appear
        height, width = edges.shape
        road_region = np.zeros_like(edges)
        
        # Create a trapezoidal mask focusing on the road area
        # This mimics the perspective view of a road (wider at bottom, narrower at top)
        pts = np.array([
            [0, height],                    # Bottom left corner
            [width * 0.2, height * 0.4],    # Top left point (narrower)
            [width * 0.8, height * 0.4],    # Top right point (narrower)
            [width, height]                 # Bottom right corner
        ], np.int32)
        
        # Fill the trapezoidal region with white pixels
        cv2.fillPoly(road_region, [pts], 255)
        
        # Combine edge detection with the road region mask
        # Only edges within the road region are considered
        road_mask = cv2.bitwise_and(edges, road_region)
        
        # Dilate the edges to connect nearby road boundaries
        # This helps create more continuous road regions
        kernel = np.ones((5, 5), np.uint8)
        road_mask = cv2.dilate(road_mask, kernel, iterations=3)
        
        return road_mask

    def region_based_road_detection(self, image):
        """
        Detect road areas based on geometric region properties and adaptive thresholding.
        
        Args:
            image: OpenCV BGR image to process
            
        Returns:
            road_mask: Binary mask where white pixels represent detected road regions
        """
        
        # Convert BGR image to grayscale for thresholding
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Apply Gaussian blur to reduce noise before thresholding
        # 5x5 kernel with standard deviation of 0
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Apply adaptive threshold for better binary conversion
        # This adapts to local image intensity variations
        thresh = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                                     cv2.THRESH_BINARY, 11, 2)
        
        # Create a trapezoidal mask for the road region (more aggressive than edge method)
        height, width = thresh.shape
        
        # Define trapezoid points for road perspective (more aggressive coverage)
        pts = np.array([
            [width * 0.05, height],         # Bottom left (almost edge)
            [width * 0.3, height * 0.5],    # Top left (mid-height)
            [width * 0.7, height * 0.5],    # Top right (mid-height)
            [width * 0.95, height]          # Bottom right (almost edge)
        ], np.int32)
        
        # Create mask and fill the trapezoidal region
        mask = np.zeros_like(thresh)
        cv2.fillPoly(mask, [pts], 255)
        
        # Apply the trapezoidal mask to the thresholded image
        # Only consider pixels within the road region
        road_mask = cv2.bitwise_and(thresh, mask)
        
        # Invert the mask to get road as white (roads are typically dark in thresholded images)
        road_mask = cv2.bitwise_not(road_mask)
        
        # Clean up the mask with morphological operations
        kernel = np.ones((3, 3), np.uint8)
        road_mask = cv2.morphologyEx(road_mask, cv2.MORPH_CLOSE, kernel)
        
        return road_mask

    def combine_road_masks(self, mask1, mask2, mask3):
        """
        Combine different road detection methods using weighted averaging.
        
        Args:
            mask1: Color-based road mask
            mask2: Edge-based road mask  
            mask3: Region-based road mask
            
        Returns:
            filtered_mask: Combined and filtered road mask
        """
        
        # Normalize all masks to 0-1 range for weighted combination
        mask1_norm = mask1.astype(np.float32) / 255.0
        mask2_norm = mask2.astype(np.float32) / 255.0
        mask3_norm = mask3.astype(np.float32) / 255.0
        
        # Weighted combination - give more weight to color-based detection (most reliable)
        # Color: 60%, Edge: 25%, Region: 15%
        combined = 0.6 * mask1_norm + 0.25 * mask2_norm + 0.15 * mask3_norm
        
        # Adaptive threshold based on image statistics
        # Calculate mean value of combined mask
        mean_val = np.mean(combined)
        # Create adaptive threshold between 0.2 and 0.5 based on image content
        threshold = max(0.2, min(0.5, mean_val + 0.1))
        
        # Apply threshold to get binary mask
        combined_binary = (combined > threshold).astype(np.uint8) * 255
        
        # Clean up with morphological operations
        kernel = np.ones((7, 7), np.uint8)
        # Close operation: fills holes in road regions
        combined_binary = cv2.morphologyEx(combined_binary, cv2.MORPH_CLOSE, kernel)
        # Open operation: removes small noise
        combined_binary = cv2.morphologyEx(combined_binary, cv2.MORPH_OPEN, kernel)
        
        # Remove small noise areas using contour analysis
        # Find all contours in the binary mask
        contours, _ = cv2.findContours(combined_binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # Create empty mask for filtered result
        filtered_mask = np.zeros_like(combined_binary)
        
        # Process each contour
        for contour in contours:
            # Calculate area of the contour
            area = cv2.contourArea(contour)
            # Only keep contours with area > 500 pixels (remove small noise)
            if area > 500:
                # Fill the contour with white
                cv2.fillPoly(filtered_mask, [contour], 255)
        
        return filtered_mask

    def create_road_visualization(self, image, road_mask):
        """
        Create and display visualization of road detection results.
        
        Args:
            image: Original camera image
            road_mask: Binary road detection mask
        """
        
        # Create colored overlay mask (same size as original image)
        colored_mask = np.zeros_like(image)
        # Set green channel to road mask (green = road)
        colored_mask[:, :, 1] = road_mask
        
        # Blend original image with colored mask
        # 70% original image + 30% green road overlay
        overlay = cv2.addWeighted(image, 0.7, colored_mask, 0.3, 0)
        
        # Calculate road percentage coverage
        # Count white pixels in road mask divided by total pixels
        road_percentage = np.sum(road_mask > 0) / (image.shape[0] * image.shape[1]) * 100
        
        # Add road percentage text to the overlay
        cv2.putText(overlay, f"Road: {road_percentage:.1f}%", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        
        # Calculate and display FPS (frames per second)
        curr_time = time.time()  # Get current time
        fps = 1 / (curr_time - self.prev_time)  # Calculate FPS
        self.prev_time = curr_time  # Update previous time
        
        # Add FPS text to the overlay
        cv2.putText(overlay, f"FPS: {fps:.2f}", (10, 70),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
        
        # Display the final result in a window
        cv2.imshow("Road Detection", overlay)
        # Wait briefly to update the window (1ms)
        cv2.waitKey(1)

def main(args=None):
    """
    Main function to initialize and run the ROS 2 road detection node.
    
    Args:
        args: Command line arguments (optional)
    """
    # Initialize ROS 2 communication
    rclpy.init(args=args)
    
    # Create the road detection node
    node = RoadOnlyNode()
    
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