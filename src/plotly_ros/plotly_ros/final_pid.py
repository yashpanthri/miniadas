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

# ─── Hyper-parameters ───────────────────────────────────────────────────
CTRL_HZ          = 20.0          # main control loop rate   [Hz]
LOOKAHEAD_SEC    = 1.0 / CTRL_HZ # preview horizon (one step)
KP, KI, KD       = 0.4, 0.01, 0.3 # PID gains   (tune!)
A_MAX, B_MAX     =  3.0,  6.0    # full-throttle / full-brake accel [m s⁻²]

class OdomTransformNode(Node):
    def __init__(self):
        super().__init__('odom_transform_node')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.odom_sub = self.create_subscription(Odometry,'/carla/hero/odometry',self.odom_callback,20)
        self.trajectory_sub = self.create_subscription(String,'trajectory_data',self.trajectory_callback,2)

        

        # Initialize variables
        self.trajectory_data = None
        self.curr_x_pos = 0.0   
        self.curr_y_pos = 0.0
        self.curr_x_vel = 0.0
        self.curr_y_vel = 0.0
        self.flag_reach_initial_x_pos = False

    def odom_callback(self, msg: Odometry):
            # Transform the odometry message to the desired frame
            pass

    def trajectory_callback(self, msg: String):
        """
        Callback to recieve trajectory data array as a string.
        """
        self.trajectory_data = msg.data
        self.get_logger().info(f"Received trajectory data string: {self.trajectory_data}")
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