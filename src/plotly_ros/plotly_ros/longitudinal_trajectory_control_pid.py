#!/usr/bin/env python3
"""
Longitudinal PID controller that tries to make the CARLA ego vehicle
match the longitudinal position/velocity contained in a reference CSV.

CSV columns:
    stamp_sec  -> time stamp in seconds (monotonic, starting at 0)
    x_pos      -> longitudinal position  [m]
    x_vel      -> longitudinal velocity  [m s⁻¹]
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from carla_msgs.msg import CarlaEgoVehicleControl
import numpy as np
import pandas as pd
from pathlib import Path

# ─── File locations ─────────────────────────────────────────────────────
_BASE = Path("/home/yashpanthri-unbuntu22/CARLA_PROJECT/mini_adas/src/plotly_ros/plotly_ros") # base directory
TRAJ_CSV = _BASE / "odometry_x_pos_and_vel.csv" # reference trajectory CSV
PID_LOG  = _BASE / "pid_trajectory_log.csv" # log file for PID controller

# ─── Hyper-parameters ───────────────────────────────────────────────────
CTRL_HZ          = 20.0          # main control loop rate   [Hz]
LOOKAHEAD_SEC    = 1.0 / CTRL_HZ # preview horizon (one step)
KP, KI, KD       = 0.4, 0.01, 0.3 # PID gains   (tune!)
A_MAX, B_MAX     =  3.0,  6.0    # full-throttle / full-brake accel [m s⁻²]

class PIDController(Node):

    def __init__(self) -> None:
        super().__init__("pid_controller")

        # ------------ 1  Load & cache reference trajectory ---------------
        traj = pd.read_csv(TRAJ_CSV)
        self.t_ref = traj["stamp_sec"].to_numpy() - traj["stamp_sec"].iloc[0]
        self.x_ref = traj["x_pos"].to_numpy() #- traj["x_pos"].iloc[0]  # zeroed at start
        self.v_ref = traj["x_vel"].to_numpy() #- traj["x_vel"].iloc[0]  # zeroed at start
        self.get_logger().info(f"First reference time: {self.t_ref[0]:.3f} s")
        self.get_logger().info(f"First reference x_pos: {self.x_ref[0]:.3f} m")
        self.get_logger().info(f"First reference x_vel: {self.x_ref[0]:.3f} m/s")

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

        # ------------ 3  Controller state --------------------------------
        self.first_sim_time   : float | None = None   # wall-clock start (sec)
        self.first_x_pos      : float | None = None   # to zero the position
        self.cur_x_pos        : float | None = None
        self.cur_x_vel        : float | None = None
        self.err_int          : float = 0.0           # integral error
        self.err_prev         : float = 0.0           # error[k-1]
        self.ts_prev          : float | None = None   # sim time[k-1]
        self.actual_x_pos   : float | None = None   # actual x position (for logging)

        # ------------ 4  Data-logging dataframe --------------------------
        self.log = pd.DataFrame(
            columns=["stamp_sec", "dist_err", "vel_err",
                     "throttle", "brake", "pid_sum", "x_pos", "x_vel"]
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
            self.get_logger().info(f"First odometry time: {self.first_sim_time:.3f} s")
            self.get_logger().info(f"First odometry x_pos: {self.first_x_pos:.3f} m")
            # self.get_logger().info(f"First odometry x_vel: {self.first_x_vel:.3f} m/s")

        # Zero the longitudinal position so log and reality start at ~0 m
        self.actual_x_pos = msg.pose.pose.position.x
        self.cur_x_pos = msg.pose.pose.position.x - self.first_x_pos
        self.cur_x_vel = msg.twist.twist.linear.x

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
        v_max    = float(np.interp(t_next, self.t_ref, self.v_ref, left=self.v_ref[0], right=self.v_ref[-1]))

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
            sim_time, dist_err, vel_err, throttle, brake, acc_cmd, self.actual_x_pos, self.cur_x_vel
        ]
        # # Write the most recent log entry to CSV
        # self.log.iloc[[-1]].to_csv(   # only the most recent row
        # PID_LOG, 
        # mode='a',          # append
        # header=not PID_LOG.exists(),  # write header only once
        # index=False,
        # )

        # Stop after we pass the last reference stamp
        if sim_time >= self.t_ref[-1]:
            self.get_logger().info("Reference complete - full brake and shutdown.")
            cmd.throttle, cmd.brake = 0.0, 1.0
            self.pub.publish(cmd)
            if(self.cur_x_vel == 0.0):
                rclpy.shutdown()

    # ────────────────────────────────────────────────────────────────────
    #  On destruction, dump the log CSV
    # ────────────────────────────────────────────────────────────────────
    def destroy_node(self) -> None:
        super().destroy_node()
        self.log.to_csv(PID_LOG, index=False)
        self.get_logger().info(f"wrote {PID_LOG}")

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
