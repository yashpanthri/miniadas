#!/usr/bin/env python3
"""
Plot error, throttle, brake and PID output versus time
from complete_log.csv (must be in the same directory).
"""

import pandas as pd
import matplotlib.pyplot as plt

# ------------------------------------------------------------------------
# 1. Load the CSV
# ------------------------------------------------------------------------
csv_file = "/home/yashpanthri-unbuntu22/CARLA_PROJECT/mini_adas/src/plotly_ros/plotly_ros/complete_log.csv"                 # <- change to your file name
df = pd.read_csv(csv_file)
print (df.head())

print("Columns:", df.columns.tolist())

# ------------------------------------------------------------------------
# 2. Plot: x-axis = first column, y-axis = one or more other columns
# ------------------------------------------------------------------------
x_col = df.columns[0]                 # e.g., "time" or "stamp_sec"
y_cols = df.columns[1:]               # plot all remaining columns

print(f"x_col: {x_col}")
print(f"y_cols: {list(y_cols)}")

plt.figure()
for col in y_cols:
    print(type(df[x_col].to_numpy().ravel()))
    plt.plot(df[x_col].to_numpy().ravel(), df[col].to_numpy().ravel(), label=col)
    print(df[col])
plt.plot(x_axis = 15)    
plt.xlabel("Time")
plt.ylabel("Value")
plt.title("CSV columns vs. " + x_col)
plt.grid(True, linestyle="--", linewidth=0.5, alpha=0.7)
plt.legend()
plt.tight_layout()
plt.show()