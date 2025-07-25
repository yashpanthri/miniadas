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
# from tf2_ros import Buffer, TransformListener
import math
import ast


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
        

        # self.tf_buffer = Buffer()
        # self.tf_listener = TransformListener(self.tf_buffer, self)
        self.trajectory_data: np.ndarray | None = None  # Initialize as None to avoid errors before data is received

        self.cmd_pub = self.create_publisher(CarlaEgoVehicleControl, '/carla/hero/vehicle_control_cmd_manual', 20)
        self.odom_sub = self.create_subscription(Odometry,'/carla/hero/odometry',self.odom_callback,20)
        self.trajectory_sub = self.create_subscription(String,'/trajectory_data',self.trajectory_callback,2)

        # Initialize variables
        self.curr_x_pos = 0.0   
        self.curr_y_pos = 0.0
        self.curr_x_vel = 0.0
        self.curr_y_vel = 0.0
        self.flag_reach_initial_x_pos = False
        self.q : Odometry.pose.pose.orientation | None = None  # Initialize as None to avoid errors before data is received

        #Declare parameters with default values
        self.declare_parameter("ctrl_hz", 20.0)
        # self.declare_parameter("lookahead_sec", LOOKAHEAD_SEC)
        self.declare_parameter("kp_pos", 0.4)
        self.declare_parameter("kp_vel", 0.4)
        self.declare_parameter("ki", 0.01)
        self.declare_parameter("kd", 0.3)
        self.declare_parameter("a_max", 3.0)
        self.declare_parameter("b_max", 6.0)

        #Read parameter values from launch file         
        self.ctrl_hz = self.get_parameter('ctrl_hz').value
        # self.lookahead_sec = self.get_parameter('lookahead_sec').value
        self.kp_pos = self.get_parameter('kp_pos').value
        self.kp_vel = self.get_parameter('kp_vel').value

        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.a_max = self.get_parameter('a_max').value
        self.b_max = self.get_parameter('b_max').value
        self.curr_pos_error = 0.0
        self.previous_vel_error = 0.0
        self.curr_vel_error = 0.0 
        self.previous_target_index = -1
        self.I_error = 0.0
        self.is_ascending: bool | None=None
        
        # rclpy.spin_once(self, timeout_sec=0.1) # Spin to allow callbacks to process
        
        # self.get_pid_control()

##############################################  CALLBACKS  ########################################################################

    def odom_callback(self, msg: Odometry):
        # Transform the odometry message to the desired frame
        self.curr_x_pos = msg.pose.pose.position.x
        self.curr_y_pos = msg.pose.pose.position.y
        self.curr_x_vel = msg.twist.twist.linear.x
        self.get_logger().info(f"X_velocity: {self.curr_x_vel}")
        self.curr_y_vel = msg.twist.twist.linear.y
        self.get_logger().info(f"Y_velocity: {self.curr_y_vel}")
        self.q = msg.pose.pose.orientation

    def trajectory_callback(self, msg: String):
        """
        Callback to recieve trajectory data array as a string.
        """
        list_of_lists = ast.literal_eval(msg.data)
        # self.get_logger().info(f"List recieved: {list_of_lists}")
        self.trajectory_data = np.array(list_of_lists, dtype=float)
        # self.get_logger().info(f"Received trajectory data string: {self.trajectory_data}")
        # self.get_logger().info(f"Received trajectory data string: {self.trajectory_data.shape}")
        # self.get_logger().info(f"Received trajectory data string: {self.trajectory_data[0][4]}")
        self.get_logger().info("Received trajectory_data string")





    @staticmethod    
    def quaternion_to_yaw(q):
        # Convert quaternion to yaw
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def current_speed(self, x_vel, y_vel):
        curr_speed = math.sqrt((x_vel)**2 + (y_vel)**2)
        self.get_logger().info(f"Speed_total: {curr_speed}")
        return curr_speed

    
    def euclidean_distance(self, x1, x2, y1, y2):
        dist = math.sqrt((x1-x2)**2 + (y1-y2)**2)
        self.get_logger().info(f"Distance to target is: {dist}")
        return dist

    def get_dot(self):              #Returns the dot product of the heading vector and the direction to the target
        # Wait until the current x position is close to the initial x position
        required_initial_x_pos = self.trajectory_data[0][0]
        required_initial_y_pos = self.trajectory_data[0][1]

        dx = required_initial_x_pos - self.curr_x_pos
        dy = required_initial_y_pos - self.curr_y_pos
        magnitude = math.hypot(dx, dy)
        dir_to_target = (dx / magnitude, dy / magnitude)

        # Get yaw
        yaw = self.quaternion_to_yaw(self.q)

        # Get heading vector from yaw
        heading = (math.cos(yaw), math.sin(yaw))
        dot = heading[0] * dir_to_target[0] + heading[1] * dir_to_target[1] # for x and y position 

        return dot # just sending x dot position coincide and not y

    
    def get_to_initial_pos(self):
        """
        Waits until the current x position is close to the initial x position.
        This is necessary to ensure that the trajectory starts from a known point.
        """
        
        while self.trajectory_data is None:
            rclpy.spin_once(self, timeout_sec=0.01)
            self.get_logger().info(f"Reached 1")
        while not self.flag_reach_initial_x_pos:
            dist = self.euclidean_distance(self.curr_x_pos, self.trajectory_data[0][0], self.curr_y_pos, self.trajectory_data[0][1])
            self.get_logger().info(f"Reached 2")
            if dist>0.5:
                self.get_logger().info(f"Is distance>0.5: {dist>0.5}")
                self.get_logger().info(f"Reached 3")
                control_cmd = CarlaEgoVehicleControl()
                control_cmd.throttle = 0.5  # Adjust throttle as needed
                control_cmd.brake = 0.0
                control_cmd.steer = 0.0
                if self.current_speed(self.curr_x_vel, self.curr_y_vel)>5.0:
                    control_cmd.throttle = 0.0
                self.get_logger().info(f"Throttle: {control_cmd.throttle}")
                self.cmd_pub.publish(control_cmd)
                rclpy.spin_once(self, timeout_sec=1/self.ctrl_hz)
            else:
                self.flag_reach_initial_x_pos = True
                self.get_logger().info(f"Reached location!")
                self.get_logger().info(f"XPOS: {self.curr_x_pos}")

                break

    '''
        prev_dist = 99999.0


        while not self.flag_reach_initial_x_pos:
            rclpy.spin_once(self, timeout_sec=0.1)  # Process callbacks

            dot = self.get_dot()  # Get the dot product to check the direction
            self.get_logger().info(f"Dot product: {dot}")

    '''
            


    '''
            if dot>0.80 and curr_dist<1 and prev_dist<curr_dist:
                self.get_logger().info(f"Reached location! Now PID Begins")
                self.flag_reach_initial_x_pos = True
                prev_dist = curr_dist
                break

            elif dot> 0.8 and curr_dist>prev_dist:
                while True:
                    control_cmd = CarlaEgoVehicleControl()
                    control_cmd.throttle = 0.0  # Adjust throttle as needed
                    control_cmd.brake = 1.0
                    control_cmd.steer = 0.0
                    self.cmd_pub.publish(control_cmd)

            else:    
                control_cmd = CarlaEgoVehicleControl()
                control_cmd.throttle = 0.5  # Adjust throttle as needed
                control_cmd.brake = 0.0
                control_cmd.steer = 0.0
                self.cmd_pub.publish(control_cmd)
                prev_dist = curr_dist
    '''            
            
        # self.get_logger().info(f"Target location reached: {self.flag_reach_initial_x_pos}")


    def find_nearest_point_index(self):     
        '''
        Not done for circular trajectory
        '''
        # min_dist = float('inf') # Assuming nearest distance as infinite
        # index = -1
        # if self.previous_target_index + 1 < self.trajectory_data.shape[0]:
        #     traj_x_pos = self.trajectory_data[self.previous_target_index+1:][0].tolist()
        #     traj_y_pos = self.trajectory_data[self.previous_target_index+1:][1].tolist()
        # for i in range(len(traj_x_pos)):
        #     d = self.euclidean_distance(self.curr_x_pos, traj_x_pos[i], self.curr_y_pos, traj_y_pos[i])
        #     if(d<min_dist):
        #         min_dist = d
        #         index = i
        # return index + self.previous_target_index + 1

        traj_x_pos = self.trajectory_data[:, 0]
        is_ascending = traj_x_pos[0] < traj_x_pos[-1] # Ascending = True, descending = False
        self.is_ascending = is_ascending
        # Case: before trajectory
        if (is_ascending and self.curr_x_pos < traj_x_pos[0]) or (not is_ascending and self.curr_x_pos > traj_x_pos[0]):
            self.previous_target_index = -1
            return -1    
        
        # Case: after trajectory
        if (is_ascending and self.curr_x_pos > traj_x_pos[-1]) or (not is_ascending and self.curr_x_pos < traj_x_pos[-1]):
            self.previous_target_index = -2
            return -2
        
        # Edge case: exactly at the last point
        if self.curr_x_pos == traj_x_pos[-1]:
            self.previous_target_index = len(traj_x_pos) - 1
            return self.previous_target_index        
        
        # Case: within bounds
        for i in range(len(traj_x_pos) - 1):
            if is_ascending:
                if traj_x_pos[i] <= self.curr_x_pos < traj_x_pos[i + 1]:
                    self.previous_target_index = i
                    return i + 1
            else:
                if traj_x_pos[i] >= self.curr_x_pos > traj_x_pos[i + 1]:
                    self.previous_target_index = i
                    return i + 1        
        

    def compute_control_error(self, index):
        target_index = index
        dt = 1/self.ctrl_hz
        if(target_index==-1):
            target_index = 0
        sign = 1.0 if self.is_ascending else -1.0
        self.curr_pos_error = sign * (self.trajectory_data[target_index][0] - self.curr_x_pos)
        self.curr_vel_error = self.current_speed(self.trajectory_data[target_index][2], self.trajectory_data[target_index][3]) - self.current_speed(self.curr_x_vel, self.curr_y_vel)
        self.I_error += self.curr_vel_error * dt
        D_error = (self.curr_vel_error - self.previous_vel_error)/dt
        self.previous_vel_error = self.curr_vel_error
        control_signal = self.kp_pos * self.curr_pos_error + self.kp_vel * self.curr_vel_error + self.ki * self.I_error + self.kd * D_error
        control_cmd = CarlaEgoVehicleControl()
        if control_signal>0:
            control_cmd.throttle = min(control_signal, 1.0)
            control_cmd.brake = 0.0
        else:
            control_cmd.throttle = 0.0
            control_cmd.brake = min(-control_signal, 1.0)
        control_cmd.steer    = 0.0
        control_cmd.gear     = 1
        self.cmd_pub.publish(control_cmd)
        






    def get_pid_control(self):
        curr_total_speed = self.current_speed(self.curr_x_vel, self.curr_y_vel)
        target_index = self.find_nearest_point_index()
        if target_index == -2:
            self.complete_brake()
        elif target_index == self.trajectory_data.shape[0]-1:
            self.complete_brake()

        self.compute_control_error(target_index)
        

            
    def complete_brake(self):
        while not (self.current_speed(self.curr_x_vel, self.curr_y_vel) == 0.0):
            self.get_logger().info(f"Braking!")
            control_cmd = control_cmd = CarlaEgoVehicleControl()
            control_cmd.throttle = 0.0  # Adjust throttle as needed
            control_cmd.brake = 1.0
            control_cmd.steer = 0.0
            rclpy.spin_once(self, timeout_sec=1/self.ctrl_hz)

    def get_pid_control_run(self):
        self.timer = self.create_timer(1.0 / self.ctrl_hz, self.get_pid_control)
        self.complete_brake()






def main():
    rclpy.init()
    node = OdomTransformNode()
    node.get_to_initial_pos()
    node.get_logger().info(f"Now time for PID")
    node.get_pid_control_run()
    # node.complete_brake()
    node.get_logger().info(f"Node shutdown!")



    try:
        rclpy.spin(node)
        
    except Exception as e:
        node.get_logger().error(f"Error occurred: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()