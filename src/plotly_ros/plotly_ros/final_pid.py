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



class OdomTransformNode(Node):
    def __init__(self):
        super().__init__('final_pid')
        self

        # self.tf_buffer = Buffer()
        # self.tf_listener = TransformListener(self.tf_buffer, self)
        self.trajectory_data: np.ndarray | None = None  # Initialize as None to avoid errors before data is received
        
        # Publisher/Subscriber objects
        self.cmd_pub = self.create_publisher(CarlaEgoVehicleControl, '/carla/hero/vehicle_control_cmd_manual', 20)
        self.odom_sub = self.create_subscription(Odometry,'/carla/hero/odometry',self.odom_callback,20)
        self.trajectory_sub = self.create_subscription(String,'/trajectory_data',self.trajectory_callback,2)
        # self.sim_dashboard_odom_pub = self.create_publisher(Odometry, '/sim_odom_data', 20)
        # self.sim_dashboard_control_pub = self.create_publisher(CarlaEgoVehicleControl, '/sim_control_data', 20)

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
        self.declare_parameter("kp_pos", 0.05) # Began with 0.4, 10.0
        self.declare_parameter("kp_vel", 6.0) # Began with 0.4, 2.0
        self.declare_parameter("ki", 0.5) # Began with 0.01, 0.01
        self.declare_parameter("kd", 0.5) # Began with 0.3, 0.1
        # self.declare_parameter("a_max", 3.0)
        # self.declare_parameter("b_max", 6.0)
        self.declare_parameter("lookahead_distance", 15.0) # Began with 5.0
        
        # Pure pursuit steering parameters
        self.declare_parameter("wheelbase", 2.875) # CARLA vehicle wheelbase in meters
        self.declare_parameter("k0", 1.0) # Base look-ahead distance in meters
        self.declare_parameter("kv", 0.2) # Speed gain for adaptive look-ahead
        self.declare_parameter("max_steering_angle", 1.39) # Max steering angle in radians (~70 degrees)

        #Read parameter values from launch file         
        self.ctrl_hz = self.get_parameter('ctrl_hz').value
        # self.lookahead_sec = self.get_parameter('lookahead_sec').value
        self.kp_pos = self.get_parameter('kp_pos').value
        self.kp_vel = self.get_parameter('kp_vel').value
        self.lookahead_distance = self.get_parameter('lookahead_distance').value

        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        # self.a_max = self.get_parameter('a_max').value
        # self.b_max = self.get_parameter('b_max').value
        
        # Pure pursuit parameters
        self.wheelbase = self.get_parameter('wheelbase').value
        self.k0 = self.get_parameter('k0').value
        self.kv = self.get_parameter('kv').value
        self.max_steering_angle = self.get_parameter('max_steering_angle').value

        self.curr_pos_error = 0.0
        self.previous_vel_error = 0.0
        self.curr_vel_error = 0.0 
        self.previous_target_index = -1
        self.I_error = 0.0
        self.is_ascending: bool | None=None        # rclpy.spin_once(self, timeout_sec=0.1) # Spin to allow callbacks to process
        self.previous_target_index = 0
        self.heading_error = 0.0
        
        # Braking state
        self.is_braking = False
        # self.get_pid_control()

##############################################  CALLBACKS  ########################################################################

    def odom_callback(self, msg: Odometry):
        # Transform the odometry message to the desired frame
        self.curr_x_pos = msg.pose.pose.position.x
        self.curr_y_pos = msg.pose.pose.position.y
        self.curr_x_vel = msg.twist.twist.linear.x
        # Remove excessive logging - only log occasionally
        # self.get_logger().info(f"X_velocity: {self.curr_x_vel}")
        self.curr_y_vel = msg.twist.twist.linear.y
        # self.get_logger().info(f"Y_velocity: {self.curr_y_vel}")
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
    
    def get_path_direction(self):
        """
        Determine the overall direction of the trajectory
        Returns: path angle in radians
        """
        if len(self.trajectory_data) < 2:
            return 0.0
        
        # Calculate overall path direction from start to end
        start_x = self.trajectory_data[0][0]
        end_x = self.trajectory_data[-1][0]
        start_y = self.trajectory_data[0][1]
        end_y = self.trajectory_data[-1][1]
        
        # Calculate path angle using atan2
        path_angle = math.atan2(end_y - start_y, end_x - start_x)
        
        return path_angle
    
    def current_speed(self, x_vel, y_vel):
        curr_speed = math.sqrt((x_vel)**2 + (y_vel)**2)
        # self.get_logger().info(f"Speed_total: {curr_speed}")
        return curr_speed

    
    def euclidean_distance(self, x1, x2, y1, y2):
        dist = math.sqrt((x1-x2)**2 + (y1-y2)**2)
        # self.get_logger().info(f"Distance to target is: {dist}")
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
            dist = self.euclidean_distance(self.curr_x_pos, self.trajectory_data[self.previous_target_index][0], self.curr_y_pos, self.trajectory_data[self.previous_target_index][1])
            self.get_logger().info(f"Reached 2")
            if dist>0.5:
                self.get_logger().info(f"Is distance>0.5: {dist>0.5}")
                self.get_logger().info(f"Reached 3")
                control_cmd = CarlaEgoVehicleControl()
                control_cmd.throttle = 0.5  # Adjust throttle as needed
                control_cmd.brake = 0.0
                control_cmd.steer = 0.0
                if self.current_speed(self.curr_x_vel, self.curr_y_vel)>3.0:
                    control_cmd.throttle = 0.0
                self.get_logger().info(f"Throttle: {control_cmd.throttle}")
                self.cmd_pub.publish(control_cmd)
                rclpy.spin_once(self, timeout_sec=1/self.ctrl_hz)
            else:
                self.flag_reach_initial_x_pos = True
                self.get_logger().info(f"Reached location!")
                self.get_logger().info(f"XPOS: {self.curr_x_pos}")

                break



    def find_lookahead_target(self):
        """
        Find the target point at lookahead_distance ahead of the vehicle
        """
        # Update lookahead distance based on current speed
        self.update_lookahead_distance()
        
        min_distance_diff = float('inf')
        target_index = self.previous_target_index
        found_ahead = False  # Flag to check if a point ahead is found

        # Get vehicle heading
        vehicle_heading_angle = self.quaternion_to_yaw(self.q)
        


        for i in range(self.previous_target_index, len(self.trajectory_data)): #len(self.trajectory_data) is the number of rows in the trajectory data
            distance = self.euclidean_distance(self.curr_x_pos, self.trajectory_data[i][0], self.curr_y_pos, self.trajectory_data[i][1])
            # Calculate angle from vehicle to this trajectory point using atan2
            dx = self.trajectory_data[i][0] - self.curr_x_pos
            dy = self.trajectory_data[i][1] - self.curr_y_pos
            target_point_angle = math.atan2(dy, dx)
            angle_diff_abs = abs(target_point_angle - vehicle_heading_angle)
            is_forward = angle_diff_abs < math.pi/2
            distance_diff = abs(distance - self.lookahead_distance)
            if distance_diff < min_distance_diff and distance > self.lookahead_distance and True:
                min_distance_diff = distance_diff
                target_index = i
                found_ahead = True  # Marked:Found a point ahead
        
        # If no point ahead found, use the last point
        if not found_ahead:
            target_index = len(self.trajectory_data) - 1
        
        if target_index >= len(self.trajectory_data) - 1:
            return len(self.trajectory_data) - 1
        
        # Debug logging
        if found_ahead:
            self.get_logger().info(f"Found lookahead target: {target_index}, adaptive distance: {self.lookahead_distance:.2f}m")
        else:
            self.get_logger().info(f"No point ahead found, using last point: {target_index}, adaptive distance: {self.lookahead_distance:.2f}m")

        return target_index

    def update_lookahead_distance(self):
        """
        Update the look-ahead distance based on the current speed
        """
        current_speed = self.current_speed(self.curr_x_vel, self.curr_y_vel)
        old_lookahead = self.lookahead_distance
        self.lookahead_distance = self.k0 + self.kv * current_speed
        self.get_logger().info(f"Lookahead distance: {self.lookahead_distance:.1f}m")
        
        # Debug logging (only log significant changes to avoid spam)
        if abs(old_lookahead - self.lookahead_distance) > 0.2:
            self.get_logger().info(f"Adaptive lookahead: {old_lookahead:.1f}m -> {self.lookahead_distance:.1f}m (speed: {current_speed:.1f}m/s)")

    def pure_pursuit_steering(self, target_index, heading_error):
        """
        Calculate steering angle using pure pursuit algorithm
        """
        if target_index >= len(self.trajectory_data): # If target index is greater than or equal to the length of the trajectory data, return 0.0 as it is time for braking
            return 0.0

            
        # 4) Calculate curvature
        kappa = 2.0 * math.sin(heading_error) / self.lookahead_distance
        
        # 5) Calculate steering angle using bicycle model
        delta = math.atan(self.wheelbase * kappa)
        
        # 6) Normalize for CARLA control message
        delta_norm = max(-1.0, min(delta / self.max_steering_angle, 1.0))
        self.get_logger().info(f"Steering angle: {delta_norm:.3f}")
        
        return delta_norm


    def compute_control_error(self, index):
        target_index = index
        dt = 1/self.ctrl_hz
        
        if(target_index==-1):
            target_index = 0
        
        # 2D position error
        target_x = self.trajectory_data[target_index][0]
        target_y = self.trajectory_data[target_index][1]
        target_speed = math.sqrt(self.trajectory_data[target_index][2]**2 + self.trajectory_data[target_index][3]**2)
        # 2D Euclidean distance error
        self.curr_pos_error = math.sqrt((target_x - self.curr_x_pos)**2 + (target_y - self.curr_y_pos)**2)  # 2D DISTANCE

        '''
        sign = 1.0 if self.is_ascending else -1.0
        self.curr_pos_error = sign * (self.trajectory_data[target_index][0] - self.curr_x_pos)
        self.curr_vel_error = self.current_speed(self.trajectory_data[target_index][2], self.trajectory_data[target_index][3]) - self.current_speed(self.curr_x_vel, self.curr_y_vel)
        '''  
        self.curr_vel_error = self.current_speed(self.trajectory_data[target_index][2], self.trajectory_data[target_index][3]) - self.current_speed(self.curr_x_vel, self.curr_y_vel)      
        self.I_error += self.curr_vel_error * dt
        D_error = (self.curr_vel_error - self.previous_vel_error)/dt
        self.previous_vel_error = self.curr_vel_error
        control_signal = self.kp_pos * self.curr_pos_error + self.kp_vel * self.curr_vel_error + self.ki * self.I_error + self.kd * D_error
        control_cmd = CarlaEgoVehicleControl()
        if control_signal>0:
            control_cmd.throttle = min(control_signal, 1.0)
            control_cmd.brake = 0.0
            if(target_speed<self.current_speed(self.curr_x_vel, self.curr_y_vel)+0.1):
                control_cmd.throttle = 0.0
        else:
            control_cmd.throttle = 0.0
            control_cmd.brake = min(-control_signal, 1.0)
        control_cmd.steer    = 0.0
        control_cmd.gear     = 1

        
        # Calculate heading error for future steering control
        dx = target_x - self.curr_x_pos
        dy = target_y - self.curr_y_pos
        target_angle = math.atan2(dy, dx)
        vehicle_heading = self.quaternion_to_yaw(self.q)
        heading_error = vehicle_heading - target_angle

        # Normalize heading error to [-π, π]
        while heading_error > math.pi:
            heading_error -= 2 * math.pi
        while heading_error < -math.pi:
            heading_error += 2 * math.pi

        # Store heading error for future use
        self.heading_error = heading_error
        
        # Calculate steering using pure pursuit
        steering_angle = self.pure_pursuit_steering(target_index, heading_error)
        control_cmd.steer = steering_angle
        self.get_logger().info(f"Steering angle: {steering_angle:.3f}")
        
        self.cmd_pub.publish(control_cmd)
        # Log heading error for debugging
        self.get_logger().info(f"Heading error: {math.degrees(heading_error):.1f}°, Steering: {steering_angle:.3f}")


    def get_pid_control(self):
        # Debug: Show that PID control is being called
        self.get_logger().info("PID control function called by timer")
        
        # Calculate current speed
        curr_total_speed = self.current_speed(self.curr_x_vel, self.curr_y_vel)

        # If already braking, continue braking
        if self.is_braking:
            self.complete_brake()
            return

        target_index = self.find_lookahead_target()
        
        # Update previous target index for next iteration
        self.previous_target_index = target_index
        
        # Check if we've reached the end of trajectory
        if target_index >= len(self.trajectory_data) - 1:
            self.get_logger().info("Reached end of trajectory, initiating brake")
            self.is_braking = True
            self.complete_brake()
            return
        else:
            self.compute_control_error(target_index)
        

            
    def complete_brake(self):
        """
        Continuously apply brakes until vehicle comes to a complete stop
        """
        # Create brake command
        control_cmd = CarlaEgoVehicleControl()
        control_cmd.throttle = 0.0
        control_cmd.brake = 1.0
        control_cmd.steer = 0.0  # Keep steering straight during braking
        control_cmd.gear = 1
        
        # Publish brake command
        self.cmd_pub.publish(control_cmd)
        
        # Log current speed
        current_speed = self.current_speed(self.curr_x_vel, self.curr_y_vel)
        self.get_logger().info(f"Braking! Current speed: {current_speed:.2f} m/s")
        
        # Check if vehicle has stopped
        if current_speed < 0.1:  # Consider stopped if speed < 0.1 m/s
            self.get_logger().info("Vehicle has come to a complete stop!")
            # Stop the timer to prevent further control commands
            if hasattr(self, 'timer'):
                self.timer.cancel()
                self.get_logger().info("PID timer stopped - vehicle stopped successfully")

    def get_pid_control_run(self):
        self.timer = self.create_timer(1.0 / self.ctrl_hz, self.get_pid_control)
        # Remove the self.complete_brake() call - it was preventing the timer from running
        self.get_logger().info(f"PID timer created with frequency: {self.ctrl_hz} Hz")


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