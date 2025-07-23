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
y_ref = ref["y_pos"].to_numpy()
y_vel = ref["y_vel"].to_numpy()

x_pid = log["x_pos"].to_numpy()
v_pid = log["x_vel"].to_numpy()

var1 =var2= True
var3 = True
if(var1 ==  True):
    # ─── Figure with twin y-axes ───────────────────────────────────────────
    fig1, ax_pos = plt.subplots(figsize=(10, 6))

    # Left y-axis: position
    ln1 = ax_pos.plot(t_ref, x_ref, label="Ref x_pos", lw=2.0)
    ln2 = ax_pos.plot(t_log, x_pid, label="PID x_pos", ls="--")
    ax_pos.set_ylabel("Longitudinal position  [m]")
    ax_pos.grid(True, ls="--", lw=0.5, alpha=0.7)

    # Combined legend
    lines  = ln1 + ln2 
    labels = [l.get_label() for l in lines]
    ax_pos.legend(lines, labels, loc="best")

    ax_pos.set_xlabel("Time since start  [s]")
    plt.title("Reference vs. PID-tracked trajectory (position)")
    plt.tight_layout()
    plt.show()

if var2 == True:
    # ─── Plot velocity on a separate y-axis ────────────────────────────────
    # Right y-axis: velocity
    fig2, ax_vel = plt.subplots(figsize=(10, 6))
    ln3 = ax_vel.plot(t_ref, v_ref, label="Ref x_vel", lw=2.0, color="tab:red")
    ln4 = ax_vel.plot(t_log, v_pid, label="PID x_vel", ls="--", color="tab:red")
    ax_vel.set_ylabel("Longitudinal velocity  [m s⁻¹]")
    # Combined legend
    lines  = ln3 + ln4
    labels = [l.get_label() for l in lines]
    ax_vel.legend(lines, labels, loc="best")
    ax_vel.set_xlabel("Time since start  [s]")
    plt.title("Reference vs. PID-tracked trajectory (velocity)")
    plt.tight_layout()
    plt.show()

# ─── Plot xy ────────────────────────────────
# # Right y-axis: velocity
# fig3,xy = plt.subplots(figsize=(10, 6))
# ln5 = xy.plot(x_ref, y_ref, label="Ref xy_pos", lw=2.0, color="tab:red")
# xy.set_ylabel("y position  [m]")
# # Combined legend
# lines  = ln5
# labels = [l.get_label() for l in lines]
# xy.legend(lines, labels, loc="best")
# xy.set_xlabel("x position  [m]")
# plt.title("XY trajectory reference")
# plt.tight_layout()
# plt.show()

# # Right y-axis: velocity
# fig4,xy = plt.subplots(figsize=(10, 6))
# ln5 = xy.plot(x_ref, t_ref, label="Ref xt_pos", lw=2.0, color="tab:red")
# ln6 = xy.plot(x_pid, t_ref, label="Ref yt_pos", ls="--", color="tab:blue")
# xy.set_ylabel("y position  [m]")
# # Combined legend
# lines  = ln5 + ln6
# labels = [l.get_label() for l in lines]
# xy.legend(lines, labels, loc="best")
# xy.set_xlabel("x position  [m]")
# plt.title("XY trajectory reference")
# plt.tight_layout()
# plt.show()



# if var3 == True:
#     width  = 10            # inches ― change this to any width you like
#     aspect = 0.6           # height = width × aspect  (≈ 60 % of the width)
#     height = width * aspect

#     fig, xy = plt.subplots(figsize=(width, height))   # <-- only width in one place

#     # marker-only scatter (no connecting line)
#     ln = xy.plot(
#         x_ref, y_ref,
#         linestyle='', marker='o', color='tab:red',
#         label='Ref xy_pos'
#     )

#     xy.set_xlabel('x position [m]')
#     xy.set_ylabel('y position [m]')
#     xy.set_title('XY trajectory reference')
#     xy.legend(ln, [l.get_label() for l in ln], loc='best')

#     xy.set_aspect('equal', adjustable='box')          # square scale (optional)
#     plt.tight_layout()
#     plt.show()

if(False):
    import plotly.express as px

    # x_ref, y_ref, t_ref should already hold your data
    fig = px.scatter(
        x=x_ref,
        y=y_ref,
        hover_data={
            "time [s]": t_ref,
            "x position [m]": x_ref,
            "y position [m]": y_ref
        },
        labels={"x": "x position [m]", "y": "y position [m]"},
        title="XY trajectory  (hover to see time & coordinates)",
    )

    fig.show()