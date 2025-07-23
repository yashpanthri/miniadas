import pandas as pd
import numpy as np
from scipy.interpolate import interp1d
np.set_printoptions(threshold=np.inf)

def load_csv(csv_path):
    """
    1. Loads the CSV file.
    2. Converts nanoseconds to seconds.
    """
    df = pd.read_csv(csv_path)
    df['timestamp_sec'] = df['timestamp_ns'] * 1e-9
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
    print(f"returning:\n {values}")
    return values


def interpolate_trajectory(df):
    """
    1. Interpolates required columns.
    2. Returns a combined NumPy array.
    """
    timestamps = df['timestamp_sec'].values
    pos_x = interpolate_column(timestamps, df['odom.pos.x'].values)
    pos_y = interpolate_column(timestamps, df['odom.pos.y'].values)
    vel_x = interpolate_column(timestamps, df['odom.vel.x'].values)
    vel_y = interpolate_column(timestamps, df['odom.vel.y'].values)

    combined = np.column_stack((pos_x, pos_y, vel_x, vel_y, timestamps))
    print(f"Interpolated array: {combined}")
    
    return combined

def save_as_string(array):
    """Saves the final NumPy array to disk."""
    file_name="/home/yashpanthri-unbuntu22/CARLA_PROJECT/mini_adas/src/plotly_ros/plotly_ros/CSV/preprocessed_trajectory.txt"
    str_array = np.array2string(array, separator=',')
    
    with open(file_name, 'w') as f:
        f.write(str_array)
    # print(f"Interpolation complete. Saved to '{file_name}' with shape {array.shape}")

def main():
    csv_path = "/home/yashpanthri-unbuntu22/CARLA_PROJECT/mini_adas/src/plotly_ros/plotly_ros/CSV/carla_merged_data.csv"  # Trajectory CSV file path
    try:
        df = load_csv(csv_path)
        result = interpolate_trajectory(df)
        save_as_string(result)
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()
