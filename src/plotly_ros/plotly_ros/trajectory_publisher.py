import pandas as pd
import numpy as np
from scipy.interpolate import interp1d

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

# Set NumPy print options for better readability
np.set_printoptions(threshold=np.inf) # Set to 'inf' to print all elements in the array

def load_csv(csv_path):
    """
    1. Loads the CSV file.
    2. Converts nanoseconds to seconds.
    """
    df = pd.read_csv(csv_path)
    df['timestamp_ns'] = df['timestamp_ns'] * 1e-9
    return df

def interpolate_column(timestamps, values):
    """
    1. Fills only NaN values using linear interpolation
    2. Leaves other values unchanged.
    """
    values = values.copy()
    nan_mask = np.isnan(values)
    valid_mask = ~nan_mask

    if valid_mask.sum() < 2:
        raise ValueError("Not enough valid points to interpolate.")

    interpolator = interp1d(
        timestamps[valid_mask],
        values[valid_mask],
        kind='linear',
        fill_value='extrapolate'
    )

    # Replace only missing values
    values[nan_mask] = interpolator(timestamps[nan_mask])
    # print(f"returning:\n {values}")
    return values


def interpolate_trajectory(df):
    """
    1. Interpolates required columns.
    2. Returns a combined NumPy array.
    """
    timestamps = df['timestamp_ns'].values
    pos_x = interpolate_column(timestamps, df['odom.pos.x'].values)
    pos_y = interpolate_column(timestamps, df['odom.pos.y'].values)
    vel_x = interpolate_column(timestamps, df['odom.vel.x'].values)
    vel_y = interpolate_column(timestamps, df['odom.vel.y'].values)

    combined = np.column_stack((pos_x, pos_y, vel_x, vel_y, timestamps))
    # print(f"Interpolated array: {combined}")
    print(f"Shape of the combined array: {combined.shape}")
    
    return combined

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

def save_as_string(array):
    """Saves the final NumPy array to disk."""
    file_name="/home/yashpanthri-unbuntu22/CARLA_PROJECT/mini_adas/src/plotly_ros/plotly_ros/CSV/preprocessed_trajectory.txt"
    str_array = np.array2string(array, separator=',', precision = 12, floatmode = 'maxprec_equal', formatter={'float_kind': lambda x: f"{x:.12f}"})
    
    with open(file_name, 'w') as f:
        f.write(str_array)
    # print(f"Interpolation complete. Saved to '{file_name}' with shape {array.shape}")
    return str_array

class TrajectoryPublisher(Node):
    def __init__(self, string_data):
        super().__init__('trajectory_publisher')
        self.pub = self.create_publisher(String, 'trajectory_data', 10) # Topic name 'trajectory_data'

        self.data_string = string_data
        self.get_logger().info("Trajectory string loaded. Publishing at 10 Hz.")

        # 10 Hz timer
        self.timer = self.create_timer(0.1, self.data_publisher)

    def data_publisher(self):
        msg = String()
        msg.data = self.data_string
        self.pub.publish(msg)
        self.get_logger().info("Published trajectory string")
        # self.get_logger().info(f"Publishing data... size={len(self.data_string)} chars")
        # self.get_logger().info(f"Data: {self.data_string[:10]}...")  # Print first 100 characters for debugging


def main():
    rclpy.init()
    csv_path = "/home/yashpanthri-unbuntu22/CARLA_PROJECT/mini_adas/src/plotly_ros/plotly_ros/CSV/carla_merged_data.csv"  # Trajectory CSV file path
    try:
        df = load_csv(csv_path)
        result = interpolate_trajectory(df)
        str_array = save_as_string(result)

        # Start ROS 2 publisher
        node = TrajectoryPublisher(str_array)
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()
