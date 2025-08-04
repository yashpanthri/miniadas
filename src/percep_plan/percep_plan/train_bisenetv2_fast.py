#!/usr/bin/env python3
"""
Fast BiSeNetV2 Training Script
This script trains a BiSeNetV2 model for road segmentation with reduced epochs for faster training.
"""

import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader
import torchvision.transforms as T
from torchvision import transforms
import cv2
import numpy as np
import os
import glob
from pathlib import Path
import matplotlib.pyplot as plt
from tqdm import tqdm
import time

# Import BiSeNetV2 model from your existing file
from yolo_perception import BiSeNetV2

class RoadSegmentationDataset(Dataset):
    """Dataset for road segmentation training"""
    
    def __init__(self, image_dir, mask_dir, transform=None, mask_transform=None):
        self.image_dir = image_dir
        self.mask_dir = mask_dir
        self.transform = transform
        self.mask_transform = mask_transform
        
        # Get all image files
        self.image_files = sorted(glob.glob(os.path.join(image_dir, "*.jpg")))
        
        print(f"Found {len(self.image_files)} images in {image_dir}")
        
    def __len__(self):
        return len(self.image_files)
    
    def __getitem__(self, idx):
        # Load image
        img_path = self.image_files[idx]
        image = cv2.imread(img_path)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        
        # Get corresponding mask path
        img_name = Path(img_path).stem
        mask_path = os.path.join(self.mask_dir, f"{img_name}_mask.png")
        
        if not os.path.exists(mask_path):
            print(f"Warning: Mask not found for {img_name}")
            # Create empty mask if not found
            mask = np.zeros((image.shape[0], image.shape[1]), dtype=np.uint8)
        else:
            mask = cv2.imread(mask_path, cv2.IMREAD_GRAYSCALE)
        
        # Apply transforms
        if self.transform:
            image = self.transform(image)
        if self.mask_transform:
            mask = self.mask_transform(mask)
        
        return image, mask

def get_transforms(image_size=(256, 256)):
    """Get transforms for images and masks"""
    
    image_transform = T.Compose([
        T.ToPILImage(),
        T.Resize(image_size),
        T.ToTensor(),
        T.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
    ])
    
    mask_transform = T.Compose([
        T.ToPILImage(),
        T.Resize(image_size, interpolation=T.InterpolationMode.NEAREST),
        T.ToTensor()
    ])
    
    return image_transform, mask_transform

def train_model(model, train_loader, val_loader, num_epochs=10, device='cpu'):
    """Train the BiSeNetV2 model with faster settings"""
    
    # Loss function and optimizer
    criterion = nn.BCEWithLogitsLoss()
    optimizer = optim.Adam(model.parameters(), lr=0.001)
    scheduler = optim.lr_scheduler.StepLR(optimizer, step_size=5, gamma=0.5)
    
    # Training history
    train_losses = []
    val_losses = []
    best_val_loss = float('inf')
    
    print(f"Training on {device}")
    print(f"Number of epochs: {num_epochs}")
    print(f"Training samples: {len(train_loader.dataset)}")
    print(f"Validation samples: {len(val_loader.dataset)}")
    
    start_time = time.time()
    
    for epoch in range(num_epochs):
        epoch_start = time.time()
        
        # Training phase
        model.train()
        train_loss = 0.0
        
        train_pbar = tqdm(train_loader, desc=f'Epoch {epoch+1}/{num_epochs} [Train]')
        for images, masks in train_pbar:
            images = images.to(device)
            masks = masks.to(device)
            
            # Forward pass
            optimizer.zero_grad()
            outputs = model(images)
            
            # Calculate loss
            loss = criterion(outputs, masks)
            
            # Backward pass
            loss.backward()
            optimizer.step()
            
            train_loss += loss.item()
            train_pbar.set_postfix({'Loss': f'{loss.item():.4f}'})
        
        train_loss /= len(train_loader)
        train_losses.append(train_loss)
        
        # Validation phase
        model.eval()
        val_loss = 0.0
        
        with torch.no_grad():
            val_pbar = tqdm(val_loader, desc=f'Epoch {epoch+1}/{num_epochs} [Val]')
            for images, masks in val_pbar:
                images = images.to(device)
                masks = masks.to(device)
                
                outputs = model(images)
                loss = criterion(outputs, masks)
                val_loss += loss.item()
                
                val_pbar.set_postfix({'Loss': f'{loss.item():.4f}'})
        
        val_loss /= len(val_loader)
        val_losses.append(val_loss)
        
        # Learning rate scheduling
        scheduler.step()
        
        # Save best model
        if val_loss < best_val_loss:
            best_val_loss = val_loss
            torch.save(model.state_dict(), 'bisenetv2_carla_best.pth')
            print(f"✓ Saved best model with validation loss: {val_loss:.4f}")
        
        epoch_time = time.time() - epoch_start
        total_time = time.time() - start_time
        
        print(f'Epoch {epoch+1}/{num_epochs} ({epoch_time:.1f}s):')
        print(f'  Train Loss: {train_loss:.4f}')
        print(f'  Val Loss: {val_loss:.4f}')
        print(f'  Learning Rate: {scheduler.get_last_lr()[0]:.6f}')
        print(f'  Total Time: {total_time:.1f}s')
        print('-' * 50)
    
    # Save final model
    torch.save(model.state_dict(), 'bisenetv2_carla_final.pth')
    
    total_training_time = time.time() - start_time
    print(f"Total training time: {total_training_time:.1f} seconds ({total_training_time/60:.1f} minutes)")
    
    return train_losses, val_losses

def plot_training_history(train_losses, val_losses):
    """Plot training history"""
    plt.figure(figsize=(10, 6))
    plt.plot(train_losses, label='Training Loss')
    plt.plot(val_losses, label='Validation Loss')
    plt.xlabel('Epoch')
    plt.ylabel('Loss')
    plt.title('BiSeNetV2 Training History')
    plt.legend()
    plt.grid(True)
    plt.savefig('training_history.png')
    plt.show()

def main():
    # Fast training configuration
    base_dir = "/home/yashpanthri-unbuntu22/CARLA_PROJECT/mini_adas/src/percep_plan/percep_plan/BiSeNetV2_carla.v6i.png-mask-semantic"
    image_size = (256, 256)
    batch_size = 4  # Smaller batch size for faster training
    num_epochs = 10  # Reduced epochs for faster training
    
    # Device configuration
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    print(f"Using device: {device}")
    
    # Get transforms
    image_transform, mask_transform = get_transforms(image_size)
    
    # Create datasets
    train_dataset = RoadSegmentationDataset(
        image_dir=os.path.join(base_dir, "train"),
        mask_dir=os.path.join(base_dir, "thresh_train"),
        transform=image_transform,
        mask_transform=mask_transform
    )
    
    val_dataset = RoadSegmentationDataset(
        image_dir=os.path.join(base_dir, "valid"),
        mask_dir=os.path.join(base_dir, "thresh_valid"),
        transform=image_transform,
        mask_transform=mask_transform
    )
    
    # Create data loaders
    train_loader = DataLoader(train_dataset, batch_size=batch_size, shuffle=True, num_workers=1)
    val_loader = DataLoader(val_dataset, batch_size=batch_size, shuffle=False, num_workers=1)
    
    print(f"Train samples: {len(train_dataset)}")
    print(f"Validation samples: {len(val_dataset)}")
    
    # Initialize model
    model = BiSeNetV2(n_classes=1).to(device)  # 1 class for binary segmentation
    
    # Train the model
    print("Starting fast training...")
    train_losses, val_losses = train_model(
        model, train_loader, val_loader, 
        num_epochs=num_epochs, device=device
    )
    
    # Plot training history
    plot_training_history(train_losses, val_losses)
    
    print("Fast training completed!")
    print("Models saved:")
    print("  - bisenetv2_carla_best.pth (best validation loss)")
    print("  - bisenetv2_carla_final.pth (final model)")
    print("  - training_history.png (training curves)")

if __name__ == "__main__":
    main() 