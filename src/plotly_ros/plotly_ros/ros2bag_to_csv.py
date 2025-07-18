#!/usr/bin/env python3
"""
Dump /carla/hero/odometry  →  CSV  (stamp_sec , x_pos)
"""

import csv
import rclpy
from rclpy.serialization import deserialize_message
from rclpy.executors import ExternalShutdownException
from rosidl_runtime_py.utilities import get_message
import rosbag2_py


# --------------------------------------------------------------------------- #
def read_bag_to_csv(bag_path: str, output_csv: str, topic_name: str) -> None:
    storage_options  = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    # map {topic ➜ type string}
    topic_types = {info.name: info.type for info in reader.get_all_topics_and_types()}
    msg_class   = get_message(topic_types[topic_name])     # nav_msgs/msg/Odometry
    first_timestamp = 0.0

    with open(output_csv, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['stamp_sec', 'x_pos', 'x_vel'])

        while reader.has_next():
            topic, raw, t_nsec = reader.read_next() # topic name, raw bytes, timestamp in nanoseconds
            if first_timestamp == 0.0:
                first_timestamp = t_nsec * 1e-9  # convert to seconds

            if topic != topic_name:
                continue        # skip other topics

            msg = deserialize_message(raw, msg_class)
            x_pos = msg.pose.pose.position.x                # full path
            x_vel = msg.twist.twist.linear.x
            writer.writerow([t_nsec * 1e-9 - first_timestamp, x_pos, x_vel])         # ns → s

    print(f'Saved {output_csv}')
# --------------------------------------------------------------------------- #


if __name__ == '__main__':
    try:
        rclpy.init()

        BAG_DIR = '/home/yashpanthri-unbuntu22/CARLA_PROJECT/mini_adas/src/plotly_ros/plotly_ros/rosbag2_2025_07_16-07_06_16'
        OUT_CSV = '/home/yashpanthri-unbuntu22/CARLA_PROJECT/mini_adas/src/plotly_ros/plotly_ros/odometry_x_pos_and_vel.csv'
        TOPIC   = '/carla/hero/odometry'

        read_bag_to_csv(BAG_DIR, OUT_CSV, TOPIC)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.shutdown()
