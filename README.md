# CARLA-ROS2-Plotly-PID: Autonomous Vehicle Control & Analysis

## My CARLA-ROS2-Plotly-PID Project: Complete Requirements & Workflow

This is my personal, step-by-step guide for running, recording, extracting, and analyzing autonomous vehicle data in CARLA using ROS 2 and Python. I wrote this to make sure I (or anyone else) can repeat the process, debug, and extend it easily.

---

## Project Overview

This project implements a comprehensive autonomous vehicle control system using:
- **CARLA Simulator** for realistic vehicle simulation
- **ROS 2** for robotics middleware and communication
- **Plotly Dash** for real-time data visualization
- **PID Controllers** for vehicle speed and steering control
- **Advanced Analytics** for performance evaluation and error analysis

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
    csv_plotly_dashboard.py          # Data comparison dashboard
    error_analysis_dashboard.py      # Error analysis dashboard
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

## 6. Quick Script Reference - How to Run Each Script

| Script | Command | Requirements | Purpose |
|--------|---------|--------------|---------|
| `combined_plotly_dashboard.py` | `python3 combined_plotly_dashboard.py` | ROS2 running, CARLA topics active | Real-time full sensor dashboard |
| `carla_odom_dash.py` | `python3 carla_odom_dash.py` | ROS2 running, `/carla/hero/odometry` topic | Simple 2D trajectory plot |
| `pid_ros2_node.py` | `python3 pid_ros2_node.py` | ROS2 running, `/carla/hero/speedometer` topic | Basic PID speed controller |
| `plot_log_file.py` | `python3 plot_log_file.py` | `complete_log.csv` file exists | Plot PID controller logs |
| `ros2_bag_to_csv_final.py` | `python3 ros2_bag_to_csv_final.py` | ROS2 bag file in directory | Extract bag data to CSV |
| `trajectory_publisher.py` | `python3 trajectory_publisher.py` | CSV trajectory file, ROS2 running | Publish trajectory from CSV |
| `final_pid.py` | `python3 final_pid.py` | ROS2 running, trajectory CSV, odometry topic | Advanced PID controller |
| `csv_plotly_dashboard.py` | `python3 csv_plotly_dashboard.py` | `carla_merged_data.csv` and `pid_merged.csv` in CSV/ | Compare CARLA vs PID data |
| `error_analysis_dashboard.py` | `python3 error_analysis_dashboard.py` | `carla_merged_data.csv` and `pid_merged.csv` in CSV/ | Error analysis and statistics |

---

## 7. Important Notes, Tips, and Troubleshooting

- **Timestamps:** For custom topics, always set `header.stamp` to current ROS time for alignment.
- **Data Normalization:** Normalize timestamps/positions in plotting code for easy comparison.
- **Bag File Size:** Bags get big! Monitor disk space.
- **Dependencies:** If I add new sensors/topics, update bag record and extraction scripts.
- **Python Virtual Environment:** Highly recommended for dependency isolation.
- **cv_bridge errors:** Make sure both ROS package and Python bindings are installed.
- **Bag extraction errors:** Check topic names/types in bag with `ros2 bag info`.
- **Plotly Dash not updating:** Make sure ROS node and Dash app run in separate threads (see scripts).

---

## 8. Example Vehicle Info (for reference)

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

## 9. References

- [CARLA Simulator Documentation](https://carla.readthedocs.io/)
- [ROS 2 Documentation](https://docs.ros.org/en/humble/index.html)
- [Plotly Dash Documentation](https://dash.plotly.com/)
- [pandas Documentation](https://pandas.pydata.org/)

---

**For any issues, check this file, the README, or my code comments!**

---

# CARLA and PID Data Analysis Dashboards

This section describes the comprehensive Plotly Dash dashboards for analyzing CARLA and PID controller data:

## Available Dashboards

### 1. Data Comparison Dashboard (`csv_plotly_dashboard.py`)
**Port: 8051** - http://localhost:8051

**Purpose:** Side-by-side comparison of CARLA and PID data with overlay visualizations.

**Features:**
- X Position vs. Time comparison
- Y Position vs. Time comparison  
- Trajectory comparison (Y vs X Position)
- GNSS coordinates vs Time
- GNSS Longitude vs Latitude
- IMU acceleration (X, Y, Z) vs Time
- Vehicle speed comparison
- Interactive plots with hover information
- Position-based time alignment

### 2. Error Analysis Dashboard (`error_analysis_dashboard.py`)
**Port: 8052** - http://localhost:8052

**Purpose:** Real-time error analysis and statistical metrics between CARLA and PID data.

**Features:**
- **Real-time Distance Error:** Euclidean distance between positions
- **Real-time Velocity Error:** Speed and velocity component differences
- **Root Mean Square Error (RMSE):** For position and velocity
- **Average Error and Standard Deviation:** Comprehensive statistical analysis
- **Error Distribution Histograms:** Visual error patterns
- **Cumulative Error Analysis:** Error accumulation over time
- **Performance Metrics:** Rolling RMSE and mean error trends
- **Correlation Analysis:** Position and speed correlation coefficients

## Quick Start

### Launch Individual Dashboards
```bash
# Data Comparison Dashboard
python csv_plotly_dashboard.py

# Error Analysis Dashboard
python error_analysis_dashboard.py
```

### Integration with ROS2 Launch System
You can integrate these dashboards with your existing ROS2 launch system by adding them to your launch files. For example, you can modify your `trajectorypublisher_finalpid_launch.py` to include these dashboards.

## Required Data Files

The dashboards expect the following CSV files in the `CSV/` directory:
- `carla_merged_data.csv` - CARLA simulation data
- `pid_merged.csv` - PID controller data

### Expected CSV Columns:
- `timestamp_ns` - Nanosecond timestamp
- `odom.pos.x`, `odom.pos.y` - Position coordinates
- `odom.vel.x`, `odom.vel.y` - Velocity components
- `speed` - Vehicle speed
- `gnss.latitude`, `gnss.longitude` - GPS coordinates
- `imu.linear.x`, `imu.linear.y`, `imu.linear.z` - IMU acceleration

## Error Analysis Metrics

The Error Analysis Dashboard provides comprehensive metrics:

### Position Errors
- **Distance RMSE:** Root Mean Square Error of Euclidean distance
- **Distance Mean Error:** Average distance error
- **Distance Standard Deviation:** Spread of distance errors
- **X/Y Position RMSE:** Individual axis error metrics

### Velocity Errors
- **Speed RMSE:** Root Mean Square Error of speed
- **Speed Mean Error:** Average speed error
- **Speed Standard Deviation:** Spread of speed errors
- **Velocity Magnitude RMSE:** Overall velocity vector error

### Statistical Analysis
- **Correlation Coefficients:** How well CARLA and PID data correlate
- **Maximum/Minimum Errors:** Peak error values
- **Error Distributions:** Histogram analysis of error patterns
- **Cumulative Errors:** Error accumulation over time

## Technical Details

### Data Alignment
Both dashboards use position-based time alignment to ensure fair comparison:
1. Find overlapping X position ranges between datasets
2. Align at a reference position in the middle of overlap
3. Apply time offset to synchronize datasets
4. Interpolate PID data to match CARLA timestamps

### Error Calculations
- **Distance Error:** `√((x_carla - x_pid)² + (y_carla - y_pid)²)`
- **Speed Error:** `speed_carla - speed_pid`
- **Velocity Error:** `√((vx_carla - vx_pid)² + (vy_carla - vy_pid)²)`
- **RMSE:** `√(mean(error²))`

### Performance Features
- Real-time data processing
- Interactive Plotly visualizations
- Responsive web interface
- Hover information and tooltips
- Export capabilities (built into Plotly)

## Use Cases

### For Researchers
- Compare controller performance against ground truth
- Analyze error patterns and trends
- Validate PID controller tuning
- Generate publication-quality plots

### For Engineers
- Debug controller behavior
- Monitor real-time performance
- Identify areas for improvement
- Validate simulation accuracy

### For Students
- Learn about error analysis methods
- Understand PID controller performance
- Visualize data relationships
- Practice statistical analysis

## Troubleshooting

### Common Issues

1. **"File not found" errors:**
   - Ensure CSV files exist in the `CSV/` directory
   - Check file permissions

2. **"No data available" plots:**
   - Verify CSV files contain required columns
   - Check for data alignment issues

3. **Port conflicts:**
   - Change ports in the script if 8051/8052 are in use
   - Kill existing processes using those ports

4. **Memory issues with large datasets:**
   - Consider downsampling data
   - Use smaller time windows for analysis

### Performance Tips
- Use smaller datasets for faster loading
- Close unused browser tabs
- Monitor system resources during analysis

## Example Output

The Error Analysis Dashboard provides:
- Real-time error plots with mean and RMSE lines
- Statistical summary panels
- Error distribution histograms
- Performance trend analysis
- Correlation metrics

## Contributing

To extend the dashboards:
1. Add new error metrics in `calculate_errors()`
2. Create new visualization callbacks
3. Update the layout with new components
4. Test with different datasets

## Dependencies

Required Python packages:
```bash
pip install dash plotly pandas numpy scipy
```

## License

This project is part of the CARLA ADAS development environment.

---

## 10. Data Sharing and Collaboration

### Project Data Repository
All project data, including simulation recordings, analysis results, and generated plots, are available for collaboration and research purposes:

**Google Drive Repository:** [CARLA ADAS Project Data](https://drive.google.com/drive/folders/140EWuKF6zUHajMQWQgXqGySRrF0JsUla?usp=sharing)

### Available Data Categories:
- **gif/** - Animated visualizations and demonstrations
- **plots/** - Static plots and analysis charts
- **recording_dataset/** - Raw simulation recordings and rosbag files
- **simulation_dataset/** - Processed simulation data and CSV files
- **video/** - Video recordings of simulations and demonstrations

### Data Usage Guidelines:
- All data is provided for research and educational purposes
- Please cite this project when using the data in publications
- For collaboration requests, please contact the project maintainer
- Data is regularly updated with new experiments and improvements

### Access Instructions:
1. Click the Google Drive link above
2. Navigate to the desired folder category
3. Download files as needed for your analysis
4. Use the provided CSV files with the analysis dashboards in this project

---

## 11. Visual Outputs and Results

This section showcases the visual outputs, plots, and demonstrations generated by the CARLA ADAS project.

### Dashboard Visualizations

#### Real-time Dashboard Simulation
![Dashboard Simulation](src/plotly_ros/plotly_ros/CSV/Video/Steering%20Videos/Video1/dashboard_graph_photos/gif/dashboard_simulation.gif)

*Real-time Plotly Dash dashboard showing live sensor data, trajectory tracking, and control metrics during CARLA simulation.*

#### Steering Simulation Demonstration
![Steering Simulation](src/plotly_ros/plotly_ros/CSV/Video/Steering%20Videos/Video1/dashboard_graph_photos/gif/mp4_steering_simulation-ezgif.com-video-to-gif-converter.gif)

*Demonstration of pure pursuit steering control algorithm in action during autonomous vehicle navigation.*

### Analysis Plots and Charts

#### Position Comparison Analysis
![Position Comparison](src/plotly_ros/plotly_ros/CSV/Video/Steering%20Videos/Video1/dashboard_graph_photos/plots/position_comparison.png)

*Side-by-side comparison of CARLA ground truth position vs. PID controller position tracking.*

#### GNSS Coordinate Analysis
![GNSS Comparison](src/plotly_ros/plotly_ros/CSV/Video/Steering%20Videos/Video1/dashboard_graph_photos/plots/gnss_comparison.png)

*GPS coordinate tracking and comparison between simulation and controller data.*

#### IMU Sensor Data Analysis
![IMU Comparison](src/plotly_ros/plotly_ros/CSV/Video/Steering%20Videos/Video1/dashboard_graph_photos/plots/IMU_comparison.png)

*Inertial Measurement Unit (IMU) acceleration data comparison and analysis.*

#### Speedometer Performance Analysis
![Speedometer Comparison](src/plotly_ros/plotly_ros/CSV/Video/Steering%20Videos/Video1/dashboard_graph_photos/plots/speedometer_comparison.png)

*Vehicle speed tracking and comparison between target and actual speeds.*

#### Error Distribution Analysis
![Error Distribution](src/plotly_ros/plotly_ros/CSV/Video/Steering%20Videos/Video1/dashboard_graph_photos/plots/error_distribution_analysis.png)

*Statistical distribution of position and velocity errors during autonomous navigation.*

#### Position and Velocity Error Analysis
![Position and Velocity Errors](src/plotly_ros/plotly_ros/CSV/Video/Steering%20Videos/Video1/dashboard_graph_photos/plots/pos_error_vel_error_vs_time.png)

*Time-series analysis of position and velocity errors throughout the simulation.*

#### Error Distribution and Velocity Analysis
![Error Distribution and Velocity](src/plotly_ros/plotly_ros/CSV/Video/Steering%20Videos/Video1/dashboard_graph_photos/plots/error_dist_and_vel.png)

*Comprehensive error analysis including distribution patterns and velocity relationships.*

#### Cumulative Error and Performance Metrics
![Cumulative Error Analysis](src/plotly_ros/plotly_ros/CSV/Video/Steering%20Videos/Video1/dashboard_graph_photos/plots/cumulative_error_and_performance%20mertrics_analysis(RMSE).png)

*Cumulative error analysis with Root Mean Square Error (RMSE) performance metrics.*

### PDF Reports

The project also generates comprehensive PDF reports for detailed analysis:

- **Error Analysis Report:** `error_analysis.pdf` - Detailed statistical analysis of controller performance
- **Dashboard Report:** `dashboard.pdf` - Complete dashboard functionality documentation
- **Pure Pursuit Output Report:** `pure_pursuit_output.pdf` - Steering control algorithm analysis

### Key Insights from Visualizations

1. **Trajectory Tracking:** The position comparison plots show how well the PID controller tracks the reference trajectory
2. **Error Patterns:** Error distribution analysis reveals systematic biases and random variations
3. **Performance Metrics:** RMSE calculations provide quantitative performance evaluation
4. **Real-time Monitoring:** GIF demonstrations show the live dashboard capabilities
5. **Sensor Integration:** IMU and GNSS plots demonstrate multi-sensor data fusion

### How to Generate These Visualizations

To generate similar visualizations for your own experiments:

1. **Run the Analysis Dashboards:**
   ```bash
   python3 csv_plotly_dashboard.py      # Port 8051
   python3 error_analysis_dashboard.py  # Port 8052
   ```

2. **Capture Screenshots:** Use browser screenshot tools or Plotly's built-in export features

3. **Record Screen:** Use screen recording software to capture GIFs of live dashboards

4. **Export Plots:** Use Plotly's export functionality or matplotlib's save features

5. **Generate Reports:** Use the PDF export features in the analysis dashboards

---

## Quick Start Guide

### Prerequisites
1. Install ROS 2 Humble
2. Install CARLA Simulator
3. Install Python dependencies
4. Clone this repository

### Basic Usage
```bash
# Start CARLA
./CarlaUE4.sh

# In another terminal, source ROS 2
source /opt/ros/humble/setup.bash

# Run the main dashboard
cd src/plotly_ros/plotly_ros
python3 combined_plotly_dashboard.py

# Run PID controller
python3 final_pid.py
```

### Advanced Usage
```bash
# Run error analysis dashboard
python3 error_analysis_dashboard.py

# Extract data from rosbag
python3 ros2_bag_to_csv_final.py

# Compare data
python3 csv_plotly_dashboard.py
```

---

## Features

- **Real-time Data Visualization** with Plotly Dash
- **PID Controller Implementation** for vehicle control
- **Comprehensive Error Analysis** with statistical metrics
- **Multi-sensor Data Fusion** (IMU, GNSS, Odometry)
- **Interactive Dashboards** for data comparison
- **ROS 2 Integration** for robotics applications
- **CARLA Simulator Support** for realistic testing
- **Data Export & Analysis** tools
- **Performance Metrics** calculation (RMSE, etc.)
- **Visual Documentation** with plots and GIFs

---

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request. For major changes, please open an issue first to discuss what you would like to change.

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- CARLA Simulator team for the excellent autonomous driving simulation platform
- ROS 2 community for the robotics middleware
- Plotly team for the interactive visualization library
- Open source community for various tools and libraries used in this project

---

** If you find this project useful, please give it a star!**