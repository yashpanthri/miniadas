#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from carla_msgs.msg import CarlaEgoVehicleControl
from nav_msgs.msg import Odometry 
from rclpy.time import Time
import pandas as pd
import numpy as np




class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')
        self.trajectory = pd.read_csv("/home/yashpanthri-unbuntu22/CARLA_PROJECT/mini_adas/src/plotly_ros/plotly_ros/odometry_x_position.csv")  
        
        self.stamp_sec = self.trajectory['stamp_sec'].to_numpy()
        self.stamp_sec -= self.stamp_sec[0]  # Normalize the timestamps to start from 0

        self.x_pos = self.trajectory['x_pos'].to_numpy()
        
        self.v_velocity = self.trajectory['velocity'].to_numpy()

        
        # self.get_logger().info(self.x_pos)
        # self.get_logger().info(self.trajectory.head())
        # self.get_logger().info("Columns: %s", self.trajectory.columns.tolist())
        # self.get_logger().info("Trajectory length: %d", len(self.trajectory))
        # self.get_logger().info("X Position: %s", self.trajectory.iloc[:,0].to_numpy())
        # self.get_logger().info("Y Position: %s", self.trajectory.iloc[:,1].to_numpy())
        self.publisher = self.create_publisher(CarlaEgoVehicleControl, 'carla/hero/vehicle_control_cmd_manual', 20)
        self.subscriber = self.create_subscription(Odometry, 'carla/hero/odometry', self.speed_callback, 20)
        self.target_value = abs(df.loc[1,'x_pos'] - df.loc[0, 'x_pos'])
        self.first_timestamp = 0.0
        self.current_x_pos = 0.0
        self.previous_x_pos = 0.0
        self.required_x_pos = 0.0
        # self.current_value = 0.0
        self.previous_error = 0.0
        self.integral = 0.0
        self.first_timestamp = self.get_clock().now().nanoseconds * 1e-9

        #PID gains
        self.Kp = 0.4#4 # Proportional gain
        self.Ki = 0.01#0.1 # Integral gain
        self.Kd = 0.3#3 # Derivative gain

        self.timer = self.create_timer(0.05, self.control_loop) # Control loop running at 20Hz


    def speed_callback(self, msg: Odometry):
        self.current_x_pos = msg.pose.pose.position.x   # Current x position from odometry
        # self.control_loop()

    def control_loop(self):

        # Calculate the current timestamp
        self.current_timestamp = round(self.get_clock().now().nanoseconds * 1e-9 - self.first_timestamp, )
        







        ######################### ERROR CALCULATION #########################





        # Calculate the error between the target and current speed
        error = self.target_value - self.current_value
        self.get_logger().info(f"Error: {error}")
        # Proportional Term in PID
        P = self.Kp * error

        # Integral Term in PID
        self.integral += error * 0.05 # Asssuming control loop is 20Hz
        I = self.Ki * self.integral

        # Derivative Term in PID
        derivative = (error - self.previous_error)/0.05 # Time step of 0.05s (20Hz)
        D = self.Kd * derivative

        # PID control signal (throttle)
        control_signal = P + I + D
        control_signal = max(0.0, min(1.0, control_signal))

        '''
        if control_signal >1.0:
            control_signal = 1.0
        '''
        brake_signal = 0.0
        if error < 0:  # If current speed is higher than target
            brake_signal = min(1.0, abs(error) * 0.5)

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
        self.previous_error = error

        # Log the control action
        self.get_logger().info(f"Speed: {self.current_value: .2f} m/s, Throttle: {control_signal: .2f}")

        # --------------- LOG TO DATAFRAME ---------------------
        global df 
        now = self.get_clock().now()                    # rclpy.time.Time
        stamp_sec = now.nanoseconds / 1e9               # float, seconds
        df.loc[len(df)] = {"stamp_sec": stamp_sec, "error": error, "throttle": control_message.throttle, "brake": control_message.brake, "pid": (P+I+D)}


def main(args = None):
    rclpy.init(args = args)
    node = PIDController()
    try:
        rclpy.spin(node)
    finally:
        # node.destory_node()
        df.to_csv("complete_log.csv", index=False)
        rclpy.shutdown()


if __name__ == '__main__':
    main()



