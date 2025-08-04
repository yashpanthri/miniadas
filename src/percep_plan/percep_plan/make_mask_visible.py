#!/usr/bin/env python3
"""
Make Mask Visible Script
This script reads mask files ending with "_mask.png" from train and valid folders,
applies adaptive threshold, and saves them in new folders with the same names.
"""

import cv2
import numpy as np
import os
import glob
from pathlib import Path

def process_mask_folder(input_folder, output_folder):
    """
    Process all mask files in a folder and save thresholded versions
    
    Args:
        input_folder (str): Path to input folder (train or valid)
        output_folder (str): Path to output folder (thresh_train or thresh_valid)
    """
    
    # Create output directory
    os.makedirs(output_folder, exist_ok=True)
    
    # Find all files ending with "_mask.png"
    mask_pattern = os.path.join(input_folder, "*_mask.png")
    mask_files = glob.glob(mask_pattern)
    
    print(f"Found {len(mask_files)} mask files in {input_folder}")
    
    for mask_path in mask_files:
        # Read the mask
        mask = cv2.imread(mask_path, cv2.IMREAD_GRAYSCALE)
        
        if mask is None:
            print(f"Could not read mask: {mask_path}")
            continue
        
        # Apply adaptive threshold
        _, thresholded = cv2.threshold(mask, 0.01, 255, cv2.THRESH_BINARY)

        # Get filename and save with same name
        filename = os.path.basename(mask_path)
        output_path = os.path.join(output_folder, filename)
        cv2.imwrite(output_path, thresholded)
        
        print(f"Processed: {filename}")

def main():
    # Define paths
    base_dir = "/home/yashpanthri-unbuntu22/CARLA_PROJECT/mini_adas/src/percep_plan/percep_plan/BiSeNetV2_carla.v6i.png-mask-semantic"
    
    # Process train folder
    train_input = os.path.join(base_dir, "train")
    train_output = os.path.join(base_dir, "thresh_train")
    print("Processing train folder...")
    process_mask_folder(train_input, train_output)
    
    # Process valid folder
    valid_input = os.path.join(base_dir, "valid")
    valid_output = os.path.join(base_dir, "thresh_valid")
    print("\nProcessing valid folder...")
    process_mask_folder(valid_input, valid_output)
    
    # Process test folder
    test_input = os.path.join(base_dir, "test")
    test_output = os.path.join(base_dir, "thresh_test")
    print("\nProcessing test folder...")
    process_mask_folder(test_input, test_output)
    
    print("\nAll processing complete!")
    print(f"Thresholded masks saved to:")
    print(f"  - {train_output}")
    print(f"  - {valid_output}")
    print(f"  - {test_output}")

if __name__ == "__main__":
    main() 