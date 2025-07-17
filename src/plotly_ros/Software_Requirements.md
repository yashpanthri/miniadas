
Start Carla without any vegetation, spawn the vehicle at a fixed position,

#Project 1:
Driving a vehicle in CARLA and record Ros2 bag file for the following topics:
'''
/carla/ego_vehicle/odometry
/carla/ego_vehicle/imu 
/carla/ego_vehicle/velocity_report
/carla/ego_vehicle/vehicle_status
/carla/ego_vehicle/gnss
/carla/hero/lidar
/carla/hero/radar_front
/carla/hero/control/set_transform
'''

Step 1: Plot all data using plotly
'''
/carla/ego_vehicle/odometry (ground truth data)
/carla/ego_vehicle/gnss (plot)
/carla/ego_vehicle/imu ()
/carla/ego_vehicle/velocity_report
/carla/ego_vehicle/vehicle_status
'''


Step 2: Extract data from Rosbag to csv files
'''
rosbag info my_bag.bag
rostopic echo -b my_bag.bag -p /carla/ego_vehicle/odometry > odometry.csv
rostopic echo -b my_bag.bag -p /carla/ego_vehicle/velocity_report > velocity.csv
rostopic echo -b my_bag.bag -p /carla/ego_vehicle/vehicle_status > steering.csv
'''

Step 3: Implement PID on imu CSV data

Step 4: Plot the implementation of PID using plotly


Inorder to move the ego vehicle :
ros2 topic echo /carla/hero/vehicle_control_manual_override         =       true

/carla/hero/control/set_transform                                   =       
Point position
	float64 x
	float64 y
	float64 z
Quaternion orientation
	float64 x 0
	float64 y 0
	float64 z 0
	float64 w 1

Use: ros2 topic pub /carla/hero/control/set_transform geometry_msgs/msg/Pose "{position: {x: 70.2, y: -134.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.9999960895574291, w: 0.002796581815395667}}"

With full acceleration the vehicle speed reaches more than 82km/hr



/carla/hero/imu                                                     =
(ros2 interface show sensor_msgs/msg/Imu)                            
# This is a message to hold data from an IMU (Inertial Measurement Unit)
#
# Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec
#
# If the covariance of the measurement is known, it should be filled in (if all you know is the
# variance of each measurement, e.g. from the datasheet, just put those along the diagonal)
# A covariance matrix of all zeros will be interpreted as "covariance unknown", and to use the
# data a covariance will have to be assumed or gotten from some other source
#
# If you have no estimate for one of the data elements (e.g. your IMU doesn't produce an
# orientation estimate), please set element 0 of the associated covariance matrix to -1
# If you are interpreting this message, please check for a value of -1 in the first element of each
# covariance matrix, and disregard the associated estimate.

std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id

geometry_msgs/Quaternion orientation
	float64 x 0
	float64 y 0
	float64 z 0
	float64 w 1
float64[9] orientation_covariance # Row major about x, y, z axes

geometry_msgs/Vector3 angular_velocity
	float64 x
	float64 y
	float64 z
float64[9] angular_velocity_covariance # Row major about x, y, z axes

geometry_msgs/Vector3 linear_acceleration
	float64 x
	float64 y
	float64 z
float64[9] linear_acceleration_covariance # Row major x, y z


###When the car is static the topic /carla/hero/imu echos at **50Hz**:
header:
  stamp:
    sec: 408
    nanosec: 186182498
  frame_id: hero/imu
orientation:
  x: -1.9072771281462425e-06
  y: 2.819807989679488e-08
  z: 0.9999960895556098
  w: 0.002796581815434189
orientation_covariance:
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
angular_velocity:
  x: -0.0
  y: 0.0
  z: -0.0
angular_velocity_covariance:
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
linear_acceleration:
  x: -3.657584966276772e-05
  y: 4.4859393710794393e-07
  z: 9.8100004196167
linear_acceleration_covariance:
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0



###GNSS
/carla/hero/gnss                                                =

(ros2 interface show sensor_msgs/msg/NavSatFix)
# Navigation Satellite fix for any Global Navigation Satellite System
#
# Specified using the WGS 84 reference ellipsoid

# header.stamp specifies the ROS time for this measurement (the
#        corresponding satellite time may be reported using the
#        sensor_msgs/TimeReference message).
#
# header.frame_id is the frame of reference reported by the satellite
#        receiver, usually the location of the antenna.  This is a
#        Euclidean frame relative to the vehicle, not a reference
#        ellipsoid.
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id

# Satellite fix status information.
NavSatStatus status
	#
	int8 STATUS_NO_FIX =  -1        #
	int8 STATUS_FIX =      0        #
	int8 STATUS_SBAS_FIX = 1        #
	int8 STATUS_GBAS_FIX = 2        #
	int8 status
	uint16 SERVICE_GPS =     1
	uint16 SERVICE_GLONASS = 2
	uint16 SERVICE_COMPASS = 4      #
	uint16 SERVICE_GALILEO = 8
	uint16 service

# Latitude [degrees]. Positive is north of equator; negative is south.
float64 latitude

# Longitude [degrees]. Positive is east of prime meridian; negative is west.
float64 longitude

# Altitude [m]. Positive is above the WGS 84 ellipsoid
# (quiet NaN if no altitude is available).
float64 altitude

# Position covariance [m^2] defined relative to a tangential plane
# through the reported position. The components are East, North, and
# Up (ENU), in row-major order.
#
# Beware: this coordinate system exhibits singularities at the poles.
float64[9] position_covariance

# If the covariance of the fix is known, fill it in completely. If the
# GPS receiver provides the variance of each measurement, put them
# along the diagonal. If only Dilution of Precision is available,
# estimate an approximate covariance from that.

uint8 COVARIANCE_TYPE_UNKNOWN = 0
uint8 COVARIANCE_TYPE_APPROXIMATED = 1
uint8 COVARIANCE_TYPE_DIAGONAL_KNOWN = 2
uint8 COVARIANCE_TYPE_KNOWN = 3

uint8 position_covariance_type



CAR IS STATIC THE topic /carla/hero/gnss echos published at **20Hz**:
header:
  stamp:
    sec: 498
    nanosec: 236183840
  frame_id: hero/gnss
status:
  status: 0
  service: 0
latitude: -0.0012036921751956697
longitude: 0.0006216064606453693
altitude: 2.0002999305725098
position_covariance:
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
position_covariance_type: 0



### For vehicle throttle control the topic used is:
/carla/hero/vehicle_control_cmd_manual                                  =

(ros2 interface show carla_msgs/msg/CarlaEgoVehicleControl)

#
# Copyright (c) 2018-2019 Intel Corporation.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
# This represents a vehicle control message sent to CARLA simulator

std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id

# The CARLA vehicle control data

# 0. <= throttle <= 1.
float32 throttle

# -1. <= steer <= 1.
float32 steer

# 0. <= brake <= 1.
float32 brake

# hand_brake 0 or 1
bool hand_brake

# reverse 0 or 1
bool reverse

# gear
int32 gear

# manual gear shift
bool manual_gear_shift



CAR IS STATIC THE topic /carla/hero/gnss echos published at **20Hz**:
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
throttle: 0.0
steer: 0.0
brake: 0.0
hand_brake: false
reverse: false
gear: 1
manual_gear_shift: false




#ROS2 BAG Recorded
'''
ros2 bag record /carla/hero/vehicle_control_cmd_manual /carla/hero/control/set_transform /carla/hero/control/set_target_velocity /carla/hero/gnss /carla/hero/imu /carla/hero/rgb_front/image /tf /carla/hero/odometry /carla/hero/radar_front /carla/hero/speedometer 
'''





Inorder to test run the bag and create the odometry topic plot:
Terminal 1: ros2 bag play rosbag2_2025_07_11-05_09_26_0.db3 
Terminal 2: python3 carla_odom_dash.py










Nodes/Code files:
1. combined_plotly_dashboard.py --> A complete dashboard of sensor data by plotly dash
2. pid_ros2_node.py --> PID implementation on a vehicle which starts from 0m/s to 15m/s speed and also stores a log of some important datas like throttle, brake, error, PID signal
3. plot_log_file.py --> Plots graphs using matplotlib




Recorded ros2 bag then ploted using the combined_plotly_dashboard.py python3 node
![alt text](image.png)
