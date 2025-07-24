#!/usr/bin/env python3
"""
Longitudinal PID controller that tries to make the CARLA ego vehicle
match the longitudinal position/velocity contained in a reference CSV.

CSV columns:
    x_pos      -> longitudinal position  [m]
    x_vel      -> longitudinal velocity  [m s⁻¹]
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from carla_msgs.msg import CarlaEgoVehicleControl
from std_msgs.msg import String
import numpy as np
import pandas as pd
from pathlib import Path
from tf2_ros import Buffer, TransformListener

# ─── File locations ─────────────────────────────────────────────────────
_BASE = Path("/home/yashpanthri-unbuntu22/CARLA_PROJECT/mini_adas/src/plotly_ros/plotly_ros") # base directory
TRAJ_CSV = _BASE / "odometry_x_pos_and_vel.csv" # reference trajectory CSV
PID_LOG  = _BASE / "pid_trajectory_log.csv" # log file for PID controller

# # ─── Hyper-parameters ───────────────────────────────────────────────────
# CTRL_HZ          = 20.0          # main control loop rate   [Hz]
# LOOKAHEAD_SEC    = 1.0 / CTRL_HZ # preview horizon (one step)
# KP, KI, KD       = 0.4, 0.01, 0.3 # PID gains   (tune!)
# A_MAX, B_MAX     =  3.0,  6.0    # full-throttle / full-brake accel [m s⁻²]

class OdomTransformNode(Node):
    def __init__(self):
        super().__init__('final_pid')
        

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cmd_pub = self.create_publisher(CarlaEgoVehicleControl, '/carla/hero/vehicle_control_cmd_manual', 10)
        self.odom_sub = self.create_subscription(Odometry,'/carla/hero/odometry',self.odom_callback,20)
        self.trajectory_sub = self.create_subscription(String,'trajectory_data',self.trajectory_callback,2)

        # Initialize variables
        self.trajectory_data = None
        self.curr_x_pos = 0.0   
        self.curr_y_pos = 0.0
        self.curr_x_vel = 0.0
        self.curr_y_vel = 0.0
        self.flag_reach_initial_x_pos = False
        self.trajectory_data: np.ndarray | None = None  # Initialize as None to avoid errors before data is received


        #Declare parameters with default values
        self.declare_parameter("ctrl_hz", 20.0)
        # self.declare_parameter("lookahead_sec", LOOKAHEAD_SEC)
        self.declare_parameter("kp", 0.4)
        self.declare_parameter("ki", 0.01)
        self.declare_parameter("kd", 0.3)
        self.declare_parameter("a_max", 3.0)
        self.declare_parameter("b_max", 6.0)

        #Read parameter values from launch file         
        self.linear_velocity = self.get_parameter('ctrl_hz').value
        # self.lookahead_sec = self.get_parameter('lookahead_sec').value
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.a_max = self.get_parameter('a_max').value
        self.b_max = self.get_parameter('b_max').value
        self.get_to_initial_pos()
        self.get_pid_control()

    def get_to_initial_pos(self):
        """
        Waits until the current x position is close to the initial x position.
        This is necessary to ensure that the trajectory starts from a known point.
        """

        # Ensure trajectory data is available
        while self.flag_reach_initial_x_pos is None:
            self.get_logger().info("Waiting for trajectory data...")
            rclpy.spin_once(self, timeout_sec=0.1) # Spin to allow callbacks to process

        # Wait until the current x position is close to the initial x position
        required_initial_x_pos = self.trajectory_data[0][0]
        self.get_logger().info(f"Waiting to reach initial x position: {required_initial_x_pos}")


        while not self.flag_reach_initial_x_pos and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.curr_x_pos>= required_initial_x_pos:
                self.flag_reach_initial_x_pos = True
                self.get_logger().info(f"Reached initial x position: {self.curr_x_pos}")
                break # Break the loop if the initial position is reached
            # Publish a control command to keep the vehicle moving
            control_cmd = CarlaEgoVehicleControl()
            control_cmd.throttle = 0.5  # Adjust throttle as needed
            control_cmd.brake = 0.0
            control_cmd.steer = 0.0
            self.cmd_pub.publish(control_cmd)




    def get_pid_control(self):
        pass
            





    def odom_callback(self, msg: Odometry):
            # Transform the odometry message to the desired frame
            self.curr_x_pos = msg.pose.pose.position.x
            self.curr_y_pos = msg.pose.pose.position.y
            self.curr_x_vel = msg.twist.twist.linear.x
            self.curr_y_vel = msg.twist.twist.linear.y

    def trajectory_callback(self, msg: String):
        """
        Callback to recieve trajectory data array as a string.
        """
        self.trajectory_data = np.fromstring(msg.data.strip('[]'), sep=',')
        # self.get_logger().info(f"Received trajectory data string: {self.trajectory_data}")
        # Process the trajectory data as needed
        pass

    


def main():
    rclpy.init()
    node = OdomTransformNode()

    try:
        rclpy.spin(node)
        
    except Exception as e:
        node.get_logger().error(f"Error occurred: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()