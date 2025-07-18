#!/usr/bin/env python3

from matplotlib import pyplot as plt
import rclpy
from rclpy.node import Node
from carla_msgs.msg import CarlaEgoVehicleControl
from nav_msgs.msg import Odometry 
from rclpy.time import Time
import pandas as pd
import numpy as np

df = pd.DataFrame(columns=["stamp_sec", "error", "throttle", "brake", "pid"])



class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')
        self.trajectory = pd.read_csv("/home/yashpanthri-unbuntu22/CARLA_PROJECT/mini_adas/src/plotly_ros/plotly_ros/odometry_x_pos_and_vel.csv")  
        self.stamp_sec = self.trajectory['stamp_sec'].to_numpy()
        self.stamp_sec = self.stamp_sec[:] - self.stamp_sec[0]  # Normalize the timestamps to start from 0
        self.x_pos = self.trajectory['x_pos'].to_numpy()
        self.x_vel = self.trajectory['x_vel'].to_numpy()
        # self.get_logger().info(self.x_pos)
        # self.get_logger().info(self.x_vel)
        # self.get_logger().info(self.trajectory.head())
        # self.get_logger().info("Columns: %s", self.trajectory.columns.tolist())
        # self.get_logger().info("Trajectory length: %d", len(self.trajectory))
        # self.get_logger().info("X Position: %s", self.trajectory.iloc[:,0].to_numpy())
        # self.get_logger().info("Y Position: %s", self.trajectory.iloc[:,1].to_numpy())
        self.publisher = self.create_publisher(CarlaEgoVehicleControl, 'carla/hero/vehicle_control_cmd_manual', 20)
        self.subscriber = self.create_subscription(Odometry, 'carla/hero/odometry', self.speed_callback, 20)


        # self.target_value = abs(df.loc[1,'x_pos'] - df.loc[0, 'x_pos'])


        self.first_timestamp = 0.0 # Initialize first timestamp for simulation start time
        self.current_x_pos = None    # Current x position from odometry
        self.time_next = 0.0    # Next timestamp in trajectory recorded data
        # self.current_x_pos = 0.0    # Current x position from odometry
        # self.current_x_vel = 0.0    # Current x velocity from odometry
        self.previous_x_pos = 0.0   # Previous x position from odometry
        self.required_x_pos = 0.0   # Required x position from trajectory at next timestamp
        self.required_max_x_vel = 0.0   # Required x velocity from trajectory at next timestamp
        self.desired_x_vel = 0.0  # Desired x velocity from trajectory at next timestamp
        # self.x_pos_shift = np.max(self.x_pos)  # Shift to ensure all x positions are positive
        self.current_timestamp = 0.0
        self.first_x_pos = None  # Flag to check if first x position is received
        

        
        # self.current_value = 0.0
        self.previous_error = 0.0
        self.integral = 0.0
        self.distance_error = 0.0
        self.velocity_error = 0.0
        self.first_timestamp = self.get_clock().now().nanoseconds * 1e-9
        self.dt = 0.0   # Time step for PID control; can be swapped out with a constant time step if needed

        #PID gains
        self.Kp = 0.4#4 # Proportional gain
        self.Ki = 0.01#0.1 # Integral gain
        self.Kd = 0.3#3 # Derivative gain

        # self.timer = self.create_timer(0.05, self.control_loop) # Control loop running at 20Hz


    def speed_callback(self, msg: Odometry):
        self.current_x_vel = msg.twist.twist.linear.x
        if self.first_timestamp == 0.0 and self.current_x_pos is None:
            self.first_timestamp = self.get_clock().now().nanoseconds * 1e-9
            self.x_pos = np.array(self.x_pos) - self.x_pos
            self.first_x_pos = msg.pose.pose.position.x
        self.current_x_pos = msg.pose.pose.position.x - self.first_x_pos  #+ self.x_pos_shift  # Current x position from odometry

        self.control_loop()
        # self.control_loop()

    def control_loop(self):
        
        # ─── 0. Sanity: wait until first odom arrives ──────────────────────────
        if self.current_x_pos is None or self.current_x_vel is None:
            return

        # Calculate the current timestamp of the simulation
        # This is the time since the simulation started
        # It is used to find the next required x position and velocity from the trajectory
        # The timestamp is normalized to start from 0
        # This is done by subtracting the first timestamp from the current timestamp
        # This is done to ensure that the timestamps in the trajectory data and odometry topic match
        self.current_timestamp = round(self.get_clock().now().nanoseconds * 1e-9 - self.first_timestamp, 3)
        
        # 1. Get the next timestamp from the trajectory 
        self.time_next = self.stamp_sec[self.stamp_sec > self.current_timestamp][0] if self.stamp_sec[self.stamp_sec > self.current_timestamp].size > 0 else self.time_next
        # Get the current x position from the trajectory based on the current timestamp
        self.required_x_pos = self.x_pos[self.stamp_sec == self.time_next][0] #+ self.x_pos_shift # Required x position from trajectory at next timestamp
        self.required_max_x_vel = self.x_vel[self.stamp_sec == self.time_next][0] # Required x velocity from trajectory data at next timestamp
        self.get_logger().info(f"Current Timestamp: {self.current_timestamp:.2f}, Next Timestamp: {self.time_next:.2f}, Required X Pos: {self.required_x_pos:.2f}, Required Max X Vel: {self.required_max_x_vel:.2f}, Current X Pos: {self.current_x_pos:.2f}, Current X Vel: {self.current_x_vel:.2f}")


        ######################### ERROR CALCULATION #########################

        # 2. Calculate the error between the required and current x position
        # x position can be negative as it comes from carla simulator, so we take absolute value
        self.distance_error = self.required_x_pos - self.current_x_pos 
        

        self.dt = round(self.time_next, 3) - self.current_timestamp

        # 3. Get the desired x velocity from the trajectory based on the current timestamp
        self.desired_x_vel = self.distance_error/self.dt if self.dt > 0 else 0.0  # Desired x velocity from trajectory at next timestamp
        if self.desired_x_vel > self.required_max_x_vel:
            self.desired_x_vel = self.required_max_x_vel
        self.velocity_error = self.desired_x_vel - self.current_x_vel  # Error in x velocity
        
        self.get_logger().info(f"Distance Error: {self.distance_error:.2f}, Velocity Error: {self.velocity_error:.2f}, Current X Pos: {self.current_x_pos:.2f}, Required X Pos: {self.required_x_pos:.2f}, Current X Vel: {self.current_x_vel:.2f}, Desired X Vel: {self.desired_x_vel:.2f}")

        # 4. Calculate the PID control signal using velocity error
        # Proportional Term in PID
        self.P = self.Kp * self.velocity_error

        # Integral Term in PID
        self.integral += self.velocity_error * self.dt # Asssuming control loop is 20Hz
        self.I = self.Ki * self.integral

        # Derivative Term in PID
        derivative = (self.velocity_error - self.previous_error)/self.dt if self.dt > 0 else 0.0
        self.D = self.Kd * derivative


        self.get_logger().info(
            f"PID Control - P: {self.P:.2f}, I: {self.I:.2f}, D: {self.D:.2f}"
        )


        # PID control signal (throttle)
        control_signal = self.P + self.I + self.D
        control_signal = max(0.0, min(1.0, control_signal))
        self.get_logger().info(f"Control Signal: {control_signal:.2f}")
        '''
        if control_signal >1.0:
            control_signal = 1.0
        '''
        brake_signal = 0.0
        if self.velocity_error < 0:  # If current speed is higher than target
            brake_signal = min(1.0, abs(self.velocity_error) * 0.5)
            # control_signal = 0.0  # No throttle if braking is required

        # Message for vehicle control
        '''
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

        '''
        control_message = CarlaEgoVehicleControl()
        control_message.throttle = control_signal 
        control_message.steer = 0.0  # Straight Driving
        control_message.brake = brake_signal # No braking
        control_message.hand_brake = False
        control_message.reverse = False
        control_message.gear = 1 # Forward Gear
        control_message.manual_gear_shift = False

        # Publish the control message
        self.publisher.publish(control_message)
        
        # Update the last error for derivative calculation
        self.previous_error = self.velocity_error

        # Log the control action
        self.get_logger().info(f"Speed: {self.current_x_vel: .2f} m/s, Throttle: {control_signal: .2f}")

        # --------------- LOG TO DATAFRAME ---------------------
        global df 
        now = self.get_clock().now()                    # rclpy.time.Time            
        df.loc[len(df)] = {"stamp_sec": self.current_timestamp, "error": self.velocity_error, "throttle": control_message.throttle, "brake": control_message.brake, "pid": (self.P+self.I+self.D)}


def main(args = None):
    rclpy.init(args = args)
    node = PIDController()
    try:
        rclpy.spin(node)
    finally:
        # node.destory_node()
        df.to_csv("/home/yashpanthri-unbuntu22/CARLA_PROJECT/mini_adas/src/plotly_ros/plotly_ros/pid_trajectory_log.csv", index=False)
        rclpy.shutdown()
        # csv_file = "/home/yashpanthri-unbuntu22/CARLA_PROJECT/mini_adas/src/plotly_ros/plotly_ros/pid_trajectory_log.csv" 
        # df = pd.read_csv(csv_file)
        # print (df.head())
        # print("Columns:", df.columns.tolist())
        # x_col = df.columns[0]                 # e.g., "time" or "stamp_sec"
        # y_cols = df.columns[1:]               # plot all remaining columns

        # print(f"x_col: {x_col}")
        # print(f"y_cols: {list(y_cols)}")

        # plt.figure()
        # for col in y_cols:
        #     print(type(df[x_col].to_numpy().ravel()))
        #     plt.plot(df[x_col].to_numpy().ravel(), df[col].to_numpy().ravel(), label=col)
        #     print(df[col])
        # plt.plot(x_axis = 15)    
        # plt.xlabel("Time")
        # plt.ylabel("Value")
        # plt.title("CSV columns vs. " + x_col)
        # plt.grid(True, linestyle="--", linewidth=0.5, alpha=0.7)
        # plt.legend()
        # plt.tight_layout()
        # plt.show()
        
if __name__ == '__main__':
    main()
