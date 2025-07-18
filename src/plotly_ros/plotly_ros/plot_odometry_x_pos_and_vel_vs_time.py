import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path
import numpy as np

#Path to the CSV file
csv_path = Path('/home/yashpanthri-unbuntu22/CARLA_PROJECT/mini_adas/src/plotly_ros/plotly_ros/odometry_x_pos_and_vel.csv')
# Read the CSV
df = pd.read_csv(csv_path)
# Inspect the columns
print("CSV Columns:", df.columns.tolist())

# Example: Plot assuming CSV has columns like 'timestamp', 'value'
# Change these column names to match your CSV
time_col = 'stamp_sec'
x_pos_col = 'x_pos'
x_vel_col = 'x_vel'

if time_col in df.columns and x_pos_col in df.columns and x_vel_col in df.columns:
    plt.figure(figsize=(10, 6))
    plt.plot(df[time_col].to_numpy(), df[x_pos_col].to_numpy(), label=x_pos_col)
    plt.plot(df[time_col].to_numpy(), df[x_vel_col].to_numpy(), label=x_vel_col)
    plt.xlabel('Time')
    plt.ylabel('Value')
    plt.title('x_pos and x_vel vs. Time')
    plt.grid(True)
    plt.legend(['x_pos', 'x_vel'])
    plt.tight_layout()
    plt.show()
else:
    print(f"Expected columns '{time_col}' and '{x_pos_col}' not found in CSV.")