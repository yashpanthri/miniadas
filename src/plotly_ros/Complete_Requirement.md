# Complete_Requirement.md

## My CARLA-ROS2-Plotly-PID Project: Complete Requirements & Workflow

This is my personal, step-by-step guide for running, recording, extracting, and analyzing autonomous vehicle data in CARLA using ROS 2 and Python. I wrote this to make sure I (or anyone else) can repeat the process, debug, and extend it easily.

---

## 1. System & Software Requirements

- **OS:** Ubuntu 20.04 or 22.04 (Linux)
- **Python:** 3.8–3.10 (I use 3.10)
- **CARLA Simulator:** 0.9.13 or compatible
- **ROS 2:** Humble Hawksbill (recommended)
- **Disk Space:** 10GB+ (bags and logs get big!)
- **RAM:** 8GB minimum (16GB+ is better)

### Install ROS 2
- [ROS 2 Humble Install Guide](https://docs.ros.org/en/humble/Installation.html)
- Always source ROS 2 in every terminal:
  ```bash
  source /opt/ros/humble/setup.bash
  ```

### Install Python Packages
```bash
pip install rclpy dash plotly pandas matplotlib numpy rosbag2_py rosidl_runtime_py opencv-python cv_bridge
```
- For `cv_bridge`:
  ```bash
  sudo apt install ros-humble-cv-bridge python3-opencv
  ```

### Download CARLA
- [CARLA Download](https://carla.org/)
- Run with minimal map/vegetation for best performance.

---

## 2. Project Directory Structure (Key Files)

```
src/plotly_ros/plotly_ros/
    combined_plotly_dashboard.py      # Main dashboard (Plotly Dash)
    carla_odom_dash.py               # Simple odometry dashboard
    pid_ros2_node.py                 # PID controller node
    plot_log_file.py                 # Matplotlib log plotter
    ros2_bag_to_csv_final.py         # Rosbag to CSV extractor
    trajectory_publisher.py          # Publishes trajectory from CSV
    final_pid.py                     # Advanced PID controller
    CSV/                             # Preprocessed and merged CSVs
    PID_longitudinal datas/          # Logs and plots for PID runs
    PID tuning plots/                # Plots and logs for PID tuning
    rosbag2_YYYY_MM_DD-HH_MM_SS/     # Example rosbag directories
    ... (other scripts and logs)
```

---

## 3. Step-by-Step Workflow

### **A. Start CARLA and Set Up the Scenario**
- Start CARLA **without any vegetation** for best performance.
- Spawn the ego vehicle at a fixed position. Example command:
  ```bash
  ros2 topic pub /carla/hero/control/set_transform geometry_msgs/msg/Pose "{position: {x: 70.1969985961914, y: -134.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.9999960895574291, w: 0.002796581815395667}}"
  ```
- Enable manual override (so you can control the car):
  ```bash
  ros2 topic echo /carla/hero/vehicle_control_manual_override
  # Should print: true
  ```

### **B. Record Data with ROS 2 Bag**
- Record all relevant topics (sensor, control, ground truth, etc.):
  ```bash
  ros2 bag record /carla/hero/vehicle_control_cmd_manual /carla/hero/control/set_transform /carla/hero/control/set_target_velocity /carla/hero/gnss /carla/hero/imu /carla/hero/rgb_front/image /tf /carla/hero/odometry /carla/hero/radar_front /carla/hero/speedometer
  ```
- **Tip:** Bags get big fast! Make sure you have enough disk space.

### **C. Visualize Data Live (Optional)**
- For a quick 2D trajectory plot:
  ```bash
  python3 carla_odom_dash.py
  # Subscribes to /carla/hero/odometry and shows (x, y) path in Plotly Dash
  ```
- For a full dashboard (positions, GNSS, IMU, speed, camera):
  ```bash
  python3 combined_plotly_dashboard.py
  # Subscribes to all main topics and shows everything in Plotly Dash
  ```

### **D. Extract Data from Rosbag to CSV**
- Use my script to extract and merge topics:
  ```bash
  python3 ros2_bag_to_csv_final.py
  # Produces carla_merged_data.csv and other topic-specific CSVs
  ```
- Or, for individual topics:
  ```bash
  rosbag info my_bag.bag
  rostopic echo -b my_bag.bag -p /carla/ego_vehicle/odometry > odometry.csv
  rostopic echo -b my_bag.bag -p /carla/ego_vehicle/velocity_report > velocity.csv
  rostopic echo -b my_bag.bag -p /carla/ego_vehicle/vehicle_status > steering.csv
  ```

### **E. Run/Analyze PID Control**
- Run my PID controller node:
  ```bash
  python3 pid_ros2_node.py
  # Implements PID on vehicle speed, logs to complete_log.csv
  ```
- Plot the PID log:
  ```bash
  python3 plot_log_file.py
  # Plots error, throttle, brake, PID output vs. time
  ```

### **F. Replay and Plot from Bag**
- To replay a bag and plot odometry:
  ```bash
  ros2 bag play rosbag2_2025_07_11-05_09_26_0.db3
  python3 carla_odom_dash.py
  ```

---

## 4. Topics and Message Types (What I Record/Use)

### **Main Topics:**
- `/carla/ego_vehicle/odometry` (nav_msgs/Odometry) — ground truth pose/velocity
- `/carla/ego_vehicle/gnss` (sensor_msgs/NavSatFix) — GPS data
- `/carla/ego_vehicle/imu` (sensor_msgs/Imu) — acceleration, angular velocity
- `/carla/ego_vehicle/velocity_report` — velocity info
- `/carla/ego_vehicle/vehicle_status` — status info
- `/carla/hero/lidar`, `/carla/hero/radar_front` — point cloud/radar
- `/carla/hero/control/set_transform` — vehicle pose setpoint
- `/carla/hero/vehicle_control_cmd_manual` (carla_msgs/CarlaEgoVehicleControl) — control commands
- `/carla/hero/rgb_front/image` (sensor_msgs/Image) — camera
- `/carla/hero/speedometer` (std_msgs/Float32) — speed

### **Custom/Convenience Topics:**
- `/sim_odom_data` (nav_msgs/Odometry) — republished odometry at control loop rate
- `/sim_control_data` (carla_msgs/CarlaEgoVehicleControl) — republished control commands

---

## 5. Node/Script Descriptions (What Each File Does)

### 1. combined_plotly_dashboard.py
- **What:** My main Plotly Dash dashboard for real-time visualization.
- **Does:** Subscribes to all main topics, plots X/Y position, GNSS, IMU, speed, and camera.
- **When to use:** For full sensor monitoring during simulation.

### 2. carla_odom_dash.py
- **What:** Lightweight Plotly Dash app for live 2D trajectory.
- **Does:** Plots (x, y) path from `/carla/hero/odometry`.
- **When to use:** For quick checks of vehicle movement.

### 3. pid_ros2_node.py
- **What:** My PID controller node for vehicle speed.
- **Does:** Subscribes to speed, publishes control, logs error/throttle/brake/PID output.
- **When to use:** For speed control experiments and PID tuning.

### 4. plot_log_file.py
- **What:** Plots PID logs using Matplotlib.
- **Does:** Loads `complete_log.csv`, plots error, throttle, brake, PID output vs. time.
- **When to use:** For offline analysis and PID tuning.

### 5. ros2_bag_to_csv_final.py
- **What:** Extracts and merges data from a ROS 2 bag into CSV.
- **Does:** Reads bag, extracts main topics, outputs `carla_merged_data.csv`.
- **When to use:** For post-processing and analysis in Python.

### 6. trajectory_publisher.py
- **What:** Publishes a preprocessed trajectory from CSV as a ROS 2 topic.
- **Does:** Loads/interpolates trajectory, publishes as string.
- **When to use:** For feeding reference trajectories to controllers.

### 7. final_pid.py
- **What:** Advanced PID controller for longitudinal control.
- **Does:** Subscribes to odometry/trajectory, waits for initial position, starts PID, can log/republish data.
- **When to use:** For more complex control experiments.

### 8. Other Utility Scripts
- **compare_pid_vs_trajectory_plot.py, plot_odometry_x_pos_and_vel_vs_time.py, etc.:**
  - For specific plotting/analysis tasks (Matplotlib/Plotly).

### 9. Data and Log Files
- **CSV/**: Preprocessed/merged CSVs for analysis.
- **PID_longitudinal datas/**, **PID tuning plots/**: Logs/plots from PID runs/tuning.
- **rosbag2_YYYY_MM_DD-HH_MM_SS/**: Recorded ROS 2 bag files.

---

## 6. Important Notes, Tips, and Troubleshooting

- **Timestamps:** For custom topics, always set `header.stamp` to current ROS time for alignment.
- **Data Normalization:** Normalize timestamps/positions in plotting code for easy comparison.
- **Bag File Size:** Bags get big! Monitor disk space.
- **Dependencies:** If I add new sensors/topics, update bag record and extraction scripts.
- **Python Virtual Environment:** Highly recommended for dependency isolation.
- **cv_bridge errors:** Make sure both ROS package and Python bindings are installed.
- **Bag extraction errors:** Check topic names/types in bag with `ros2 bag info`.
- **Plotly Dash not updating:** Make sure ROS node and Dash app run in separate threads (see scripts).

---

## 7. Example Vehicle Info (for reference)

```
ros2 topic echo /carla/hero/vehicle_info
# Example output:
id: 47
type: vehicle.dodge.charger_police_2020
rolename: hero
wheels:
  ...
max_rpm: 8000.0
mass: 1920.0
drag_coefficient: 0.3
center_of_mass:
  x: 0.099...
  y: 0.0
  z: -0.199...
```

---

## 8. References

- [CARLA Simulator Documentation](https://carla.readthedocs.io/)
- [ROS 2 Documentation](https://docs.ros.org/en/humble/index.html)
- [Plotly Dash Documentation](https://dash.plotly.com/)
- [pandas Documentation](https://pandas.pydata.org/)

---

**For any issues, check this file, the README, or my code comments!** 