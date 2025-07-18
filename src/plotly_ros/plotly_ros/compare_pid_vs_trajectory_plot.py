#!/usr/bin/env python3
from pathlib import Path
import pandas as pd
import matplotlib.pyplot as plt

# ─── File locations ────────────────────────────────────────────────────
_BASE    = Path("/home/yashpanthri-unbuntu22/CARLA_PROJECT/mini_adas/src/plotly_ros/plotly_ros")
TRAJ_CSV = _BASE / "odometry_x_pos_and_vel.csv"
PID_LOG  = _BASE / "pid_trajectory_log.csv"

# ─── Load data ─────────────────────────────────────────────────────────
ref = pd.read_csv(TRAJ_CSV)
log = pd.read_csv(PID_LOG)

# Convert to NumPy right here so we never hand pandas objects to matplotlib
t_ref = (ref["stamp_sec"] - ref["stamp_sec"].iloc[0]).to_numpy()
t_log = (log["stamp_sec"] - log["stamp_sec"].iloc[0]).to_numpy()

x_ref = ref["x_pos"].to_numpy()
v_ref = ref["x_vel"].to_numpy()
x_pid = log["x_pos"].to_numpy()
v_pid = log["x_vel"].to_numpy()

# ─── Figure with twin y-axes ───────────────────────────────────────────
fig, ax_pos = plt.subplots(figsize=(10, 6))

# Left y-axis: position
ln1 = ax_pos.plot(t_ref, x_ref, label="Ref x_pos", lw=2.0)
ln2 = ax_pos.plot(t_log, x_pid, label="PID x_pos", ls="--")
ax_pos.set_ylabel("Longitudinal position  [m]")
ax_pos.grid(True, ls="--", lw=0.5, alpha=0.7)

# Right y-axis: velocity
ax_vel = ax_pos.twinx()
ln3 = ax_vel.plot(t_ref, v_ref, label="Ref x_vel", lw=2.0, color="tab:red")
ln4 = ax_vel.plot(t_log, v_pid, label="PID x_vel", ls="--", color="tab:red")
ax_vel.set_ylabel("Longitudinal velocity  [m s⁻¹]")

# Combined legend
lines  = ln1 + ln2 + ln3 + ln4
labels = [l.get_label() for l in lines]
ax_pos.legend(lines, labels, loc="best")

ax_pos.set_xlabel("Time since start  [s]")
plt.title("Reference vs. PID-tracked trajectory (position & velocity)")
plt.tight_layout()
plt.show()
