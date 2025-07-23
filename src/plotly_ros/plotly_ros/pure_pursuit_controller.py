#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from carla_msgs.msg import CarlaEgoVehicleControl
import numpy as np
import pandas as pd
from pathlib import Path
from std_msgs.msg import Float32



# ─── File locations ─────────────────────────────────────────────────────
_BASE = Path("/home/yashpanthri-unbuntu22/CARLA_PROJECT/mini_adas/src/plotly_ros/plotly_ros")
TRAJ_CSV = _BASE / "traj.csv"
PP_LOG  = _BASE / "pp_log.csv"

# ─── Hyper-parameters ───────────────────────────────────────────────────
CTRL_HZ          = 20.0          # main control loop rate   [Hz]
MAX_STEER_ANGLE = 0.61 # max steering angle in radians
MAX_LD = 25.0 # max look-ahead distance [m]
MIN_LD = 5.0  # min look-ahead distance [m]
WORLD_FRAME = "world"  # world frame for the trajectory
EGO_FRAME  = "base_link"  # frame of the ego vehicle

LOOKAHEAD_SEC    = 1.0 / CTRL_HZ # preview horizon (one step)
KP, KI, KD       = 0.4, 0.01, 0.3 # PID gains   (tune!)
A_MAX, B_MAX     =  3.0,  6.0    # full-throttle / full-brake accel [m s⁻²]

class PIDController(Node):

    def __init__(self) -> None:
        super().__init__("pid_controller")

        # ------------ 1  Load & cache reference trajectory ---------------
        traj = pd.read_csv(TRAJ_CSV)
        self.t_ref = traj["stamp_sec"].to_numpy() - traj["stamp_sec"].iloc[0] # Trajectory time in seconds
        self.x_ref = traj["x_pos"].to_numpy() # Trajectory longitudinal position in meters
        self.y_ref = traj["y_pos"].to_numpy() # Trajectory lateral position in meters ===========================================
        self.v_x_ref = traj["x_vel"].to_numpy() # Trajectory longitudinal velocity in m/
        self.v_y_ref = traj["y_vel"].to_numpy() # Trajectory lateral velocity in m/s ===========================================

        # ------------ 2  ROS I/O -----------------------------------------
        self.pub = self.create_publisher(
            CarlaEgoVehicleControl,
            "/carla/hero/vehicle_control_cmd_manual",
            20,
        )
        self.sub = self.create_subscription(
            Odometry,
            "/carla/hero/odometry",
            self.odom_cb,
            20,
        )
        # ===========================================
        self.sub_speed = self.create_subscription(
            Float32
            "/carla/hero/speedometer",
            self.speed_cb,
            20,
        )

        # ------------ 3  Controller state --------------------------------
        self.first_sim_time   : float | None = None   # wall-clock start (sec) or simulation start time(conception of first message on the topic /carla/hero/odometry)
        self.first_x_pos      : float | None = None   # to zero the position or simulation start x position(first x position message on the topic /carla/hero/odometry)
        self.first_y_pos      : float | None = None   # to zero the position or simulation start y position(first y position message on the topic /carla/hero/odometry)        # ===========================================


        self.cur_x_pos        : float | None = None
        self.cur_x_vel        : float | None = None
        self.cur_y_pos        : float | None = None        # ===========================================
        self.cur_y_vel        : float | None = None        # ===========================================

        self.curr_speed       : float | None = None   # Current wheel speed of the vehicle        # ===========================================



        #==============================================figure it out ==============================================
        self.err_int          : float = 0.0           # integral error
        self.err_prev         : float = 0.0           # error[k-1]
        self.ts_prev          : float | None = None   # sim time[k-1]

        # ------------ 4  Data-logging dataframe --------------------------
        self.log = pd.DataFrame(
            columns=["stamp_sec", "dist_err", "vel_err",
                     "throttle", "brake", "pid_sum", "x_pos", "x_vel", "y_pos", "y_vel", "speed", "steer"] # Added y_pos(curr_y_pos), y_vel(curr_y_vel), speed(curr_speed), steer for logging        # ===========================================
        )

        # Main loop timer (runs only when odom is flowing)
        self.timer = self.create_timer(1.0 / CTRL_HZ, self.loop)

    # ────────────────────────────────────────────────────────────────────
    #  Odometry callback – keeps latest measurement in memory
    # ────────────────────────────────────────────────────────────────────
    def odom_cb(self, msg: Odometry) -> None:
        if self.first_sim_time is None:
            self.first_sim_time = self.get_clock().now().nanoseconds * 1e-9
            self.first_x_pos    = msg.pose.pose.position.x
            self.first_y_pos    = msg.pose.pose.position.y

        # Zero the longitudinal position so log and reality start at ~0 m
        self.cur_x_pos = msg.pose.pose.position.x - self.first_x_pos
        self.cur_x_vel = msg.twist.twist.linear.x
        self.cur_y_pos = msg.pose.pose.position.y - self.first_y_pos
        self.cur_y_vel = msg.twist.twist.linear.y


    #==============================================figure it out ==============================================
    # ────────────────────────────────────────────────────────────────────
    #  Odometry callback – keeps latest measurement in memory
    # ────────────────────────────────────────────────────────────────────
    def speed_cb(self, msg: Float32) -> None:
        self.curr_speed = msg.data




    #==============================================figure it out ==============================================



    def quaternion_to_euler(self, q):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw).
        :param q: Quaternion as a list or tuple (x, y, z, w)
        :return: Euler angles as a tuple (roll, pitch, yaw) in radians
        """
        x, y, z, w = q
        roll = np.arctan2(2.0 * (y * z + w * x), w * w - x * x - y * y + z * z)
        pitch = np.arcsin(-2.0 * (x * z - w * y))
        yaw = np.arctan2(2.0 * (x * y + w * z), w * w + x * x - y * y - z * z)
        return roll, pitch, yaw

    def get_target_point(self, l_d: float, pos: tuple, pos_theta: float):
        """
        Get the target point at a distance l_d from the current position pos with orientation pos_theta.
        :param l_d: distance to the target point
        :param pos: current position (x, y)
        :param pos_theta: current orientation in radians
        :return: target point (x, y)
        """
        x_tp = pos[0] + l_d * np.cos(pos_theta) 
        y_tp = pos[1] + l_d * np.sin(pos_theta)
        return x_tp, y_tp
    






    

    #==============================================figure it out ==============================================
    # ────────────────────────────────────────────────────────────────────
    #  Main control loop (fixed rate, but skips if odom not yet ready)
    # ────────────────────────────────────────────────────────────────────
    def loop(self) -> None: 
        if self.cur_x_pos is None or self.cur_x_vel is None:
            return  # wait for first odom

        # ---------- 1  Simulation time & look-ahead reference ------------
        sim_time = self.get_clock().now().nanoseconds * 1e-9 - self.first_sim_time
        t_next   = sim_time + LOOKAHEAD_SEC

        # Interpolated reference pose/vel at the look-ahead instant
        '''
        np.interp(x, xp, fp, left, right):
        This is a NumPy function that performs 1D linear interpolation.
        It finds the interpolated value at x based on known data points (xp, fp).
        If x is less than the smallest xp, return left.
        If x is greater than the largest xp, return right.
        '''
        x_next   = float(np.interp(t_next, self.t_ref, self.x_ref, left=self.x_ref[0], right=self.x_ref[-1]))
        v_max    = float(np.interp(t_next, self.t_ref, self.v_x_ref, left=self.v_x_ref[0], right=self.v_x_ref[-1]))

        # ---------- 2  Errors & desired velocity -------------------------
        dist_err = x_next - self.cur_x_pos          # [m]
        v_des    = dist_err / LOOKAHEAD_SEC         # [m s⁻¹]
        v_des    = np.clip(v_des, -v_max, v_max)    # don’t command faster than log
        vel_err  = v_des - self.cur_x_vel

        # ---------- 3  PID (on velocity error) ---------------------------
        dt = (sim_time - self.ts_prev) if self.ts_prev else 1.0 / CTRL_HZ
        self.err_int  += vel_err * dt
        d_err          = (vel_err - self.err_prev) / dt if self.ts_prev else 0.0
        acc_cmd        = KP*vel_err + KI*self.err_int + KD*d_err

        self.err_prev  = vel_err
        self.ts_prev   = sim_time

        # ---------- 4  Throttle / brake mapping --------------------------
        if acc_cmd >= 0.0:                    # need to speed up
            throttle = np.clip(acc_cmd / A_MAX, 0.0, 1.0)
            brake    = 0.0
        else:                                 # need to slow down
            throttle = 0.0
            brake    = np.clip(-acc_cmd / B_MAX, 0.0, 1.0)

        # ---------- 5  Publish to CARLA ----------------------------------
        cmd = CarlaEgoVehicleControl()
        cmd.throttle = float(throttle)
        cmd.brake    = float(brake)
        cmd.steer    = 0.0
        cmd.gear     = 1
        self.pub.publish(cmd)

        # ---------- 6  Logging -------------------------------------------
        self.log.loc[len(self.log)] = [
            sim_time, dist_err, vel_err, throttle, brake, acc_cmd, self.cur_x_pos, self.cur_x_vel
        ]

        # Stop after we pass the last reference stamp
        if sim_time >= self.t_ref[-1]:
            self.get_logger().info("Reference complete - full brake and shutdown.")
            cmd.throttle, cmd.brake = 0.0, 1.0
            self.pub.publish(cmd)
            rclpy.shutdown()

    # ────────────────────────────────────────────────────────────────────
    #  On destruction, dump the log CSV
    # ────────────────────────────────────────────────────────────────────
    def destroy_node(self) -> None:
        super().destroy_node()
        self.log.to_csv(PID_LOG, index=False)
        self.get_logger().info(f"✓ wrote {PID_LOG}")

# ─── Main ───────────────────────────────────────────────────────────────
def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = PIDController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
