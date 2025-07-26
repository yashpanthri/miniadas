import rclpy
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import pandas as pd

# === SETTINGS ===
# bag_path = '/home/yashpanthri-unbuntu22/CARLA_PROJECT/mini_adas/src/plotly_ros/plotly_ros/rosbag2_2025_07_26-00_38_56'  # <-- Change this to your bag folder path
bag_path = '/home/yashpanthri-unbuntu22/CARLA_PROJECT/mini_adas/src/plotly_ros/plotly_ros/rosbag2_2025_07_26-03_26_19' # ros2 sim data

topics_to_extract = [
    "/carla/hero/speedometer",
    "/carla/hero/imu",
    "/carla/hero/odometry",
    "/carla/hero/vehicle_control_cmd_manual",
    "/carla/hero/gnss"
]

# === ROS Init ===
rclpy.init()

# === Open ROS 2 Bag ===
reader = rosbag2_py.SequentialReader()
storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
converter_options = rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
reader.open(storage_options, converter_options)

# === Discover Available Topics ===
available_topics = {info.name: info.type for info in reader.get_all_topics_and_types()}

found_topics = []
not_found_topics = []

type_map = {}
for topic in topics_to_extract:
    if topic in available_topics:
        type_map[topic] = get_message(available_topics[topic])
        found_topics.append(topic)
    else:
        not_found_topics.append(topic)

# === Log topic status ===
print(f"{len(found_topics)} topic(s) found:")
for t in found_topics:
    print(f"  - {t}")

if not_found_topics:
    print(f"\n{len(not_found_topics)} topic(s) NOT found:")
    for t in not_found_topics:
        print(f"  - {t}")

if not found_topics:
    print("\nNone of the requested topics were found in the bag. Exiting.")
    rclpy.shutdown()
    exit(1)

# === Extract Messages ===
data_dict = {topic: [] for topic in found_topics}

while reader.has_next():
    topic, data, timestamp = reader.read_next()
    if topic not in found_topics:
        continue

    msg_type = type_map[topic]
    msg = deserialize_message(data, msg_type)
    entry = {"timestamp_ns": timestamp}

    # --- Custom field extraction ---
    if topic == "/carla/hero/speedometer":
        entry["speed"] = msg.data

    elif topic == "/carla/hero/imu":
        entry.update({
            "imu.linear.x": msg.linear_acceleration.x,
            "imu.linear.y": msg.linear_acceleration.y,
            "imu.linear.z": msg.linear_acceleration.z,
            "imu.angular.x": msg.angular_velocity.x,
            "imu.angular.y": msg.angular_velocity.y,
            "imu.angular.z": msg.angular_velocity.z
        })

    elif topic == "/carla/hero/odometry":
        entry.update({
            "odom.pos.x": msg.pose.pose.position.x,
            "odom.pos.y": msg.pose.pose.position.y,
            "odom.pos.z": msg.pose.pose.position.z,
            "odom.vel.x": msg.twist.twist.linear.x,
            "odom.vel.y": msg.twist.twist.linear.y,
            "odom.vel.z": msg.twist.twist.linear.z
        })

    elif topic == "/carla/hero/vehicle_control_cmd_manual":
        entry.update({
            "cmd.throttle": msg.throttle,
            "cmd.brake": msg.brake,
            "cmd.steer": msg.steer
        })

    elif topic == "/carla/hero/gnss":
        entry.update({
            "gnss.latitude": msg.latitude,
            "gnss.longitude": msg.longitude,
            "gnss.altitude": msg.altitude
        })

    data_dict[topic].append(entry)

# === Create DataFrames ===
dfs = []
for topic, records in data_dict.items():
    df = pd.DataFrame(records)
    df['timestamp_sec'] = df['timestamp_ns'] * 1e-9
    dfs.append(df.set_index('timestamp_ns'))

# === Merge and Save ===
merged = pd.concat(dfs, axis=1).sort_index().reset_index()
# merged.to_csv("/home/yashpanthri-unbuntu22/CARLA_PROJECT/mini_adas/src/plotly_ros/plotly_ros/CSV/carla_merged_data.csv", index=False)
merged.to_csv("/home/yashpanthri-unbuntu22/CARLA_PROJECT/mini_adas/src/plotly_ros/plotly_ros/CSV/pid_merged.csv", index=False)
# print(f"\nMerged CSV saved: carla_merged_data.csv")
print(f"\nMerged CSV saved: pid_merged.csv")

# === Shutdown ROS ===
rclpy.shutdown()