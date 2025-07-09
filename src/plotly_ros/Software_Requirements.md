
Start Carla without any vegetation, spawn the vehicle at a fixed position,

#Project 1:
Driving a vehicle in CARLA and record Ros2 bag file for the following topics:
'''
/carla/ego_vehicle/odometry
/carla/ego_vehicle/imu 
/carla/ego_vehicle/velocity_report
/carla/ego_vehicle/vehicle_status
/carla/ego_vehicle/gnss
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