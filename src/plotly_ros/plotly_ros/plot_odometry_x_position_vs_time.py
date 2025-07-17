import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path
import numpy as np

#Path to the CSV file
csv_path = Path('/home/yashpanthri-unbuntu22/CARLA_PROJECT/mini_adas/src/plotly_ros/plotly_ros/odometry_x_position.csv')
# Read the CSV
df = pd.read_csv(csv_path)
# Inspect the columns
print("CSV Columns:", df.columns.tolist())

# Example: Plot assuming CSV has columns like 'timestamp', 'value'
# Change these column names to match your CSV
time_col = 'stamp_sec'
value_col = 'x_pos'

if time_col in df.columns and value_col in df.columns:
    plt.figure(figsize=(10, 6))
    plt.plot(df[time_col].to_numpy(), df[value_col].to_numpy(), label=value_col)
    plt.xlabel('Time')
    plt.ylabel('Value')
    plt.title(f'{value_col} vs. Time')
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()
else:
    print(f"Expected columns '{time_col}' and '{value_col}' not found in CSV.")