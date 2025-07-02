# Mini-ADAS Stack 🚗🦾  
**Camera-only Perception · Control · Planning**  
*Project kick-off: 21 Jun 2025*

> An end-to-end practice platform that fuses CARLA, ROS 2 Humble, OpenCV, and custom nodes
> for perception, control, and motion-planning research.  
> This README logs daily milestones—**Day 1** establishes the environment.

---

## Table of Contents

1. [System Requirements](#system-requirements)
2. [Quick-Start (TL;DR)](#quick-start)
3. [Full Installation](#full-installation)
   - [3.1 Host Packages](#31-host-packages)
   - [3.2 ROS 2 Humble](#32-ros-2-humble)
   - [3.3 CARLA 0.9.15](#33-carla-0915)
   - [3.4 Workspace Build](#34-workspace-build)
   - [3.5 Python Extras](#35-python-extras)
4. [Launch Sequence](#launch-sequence)
5. [Verification Tests](#verification-tests)
6. [Day-1 Deliverables](#day-1-deliverables)
7. [Troubleshooting](#troubleshooting)
8. [References](#references)

---

## System Requirements

| Item | Minimum Spec | Current Setup |
|------|--------------|---------------|
| **OS** | Ubuntu 22.04 LTS | Ubuntu 22.04.5 LTS ✅ |
| **CPU** | Quad core 3 GHz | AMD Ryzen 5 5600X ✅ |
| **GPU** | NVIDIA GTX 1060 6 GB (Driver ≥ 470) | NVIDIA RTX 2060 6GB (Driver 535.230.02) ✅ |
| **RAM** | 16 GB | 16 GB ✅ |
| **Disk** | 30 GB free | 30+ GB available ✅ |
| **Python** | 3.10 (system default) | Python 3.10 ✅ |

---

## Quick-Start

```bash
# clone repo (if starting fresh)
git clone https://github.com/<you>/mini_adas_stack.git
cd mini_adas_stack

# run one-liner to install ROS 2 Humble + CARLA 0.9.15
./scripts/install_all.sh

# open 4 terminals (see Launch Sequence) and drive!
```

For granular steps, keep reading.

---

## Full Installation

### 3.1 Host Packages

```bash
sudo apt update
sudo apt install -y build-essential git curl gnupg lsb-release \
                    software-properties-common wget unzip
# (optional: NVIDIA driver)
sudo ubuntu-drivers autoinstall   # -> reboot
```

### 3.2 ROS 2 Humble

```bash
# add key + repo
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
     -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
https://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
| sudo tee /etc/apt/sources.list.d/ros2.list

sudo apt update
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 3.3 CARLA 0.9.15

```bash
export PROOT=$HOME/CARLA_PROJECT
mkdir -p $PROOT && cd $PROOT

wget https://github.com/carla-simulator/carla/releases/download/0.9.15/CARLA_0.9.15.tar.gz
tar -xzf CARLA_0.9.15.tar.gz         # extracts to CARLA_0.9.15/

echo "export CARLA_ROOT=$PROOT/CARLA_0.9.15"     >> ~/.bashrc
echo "export PYTHONPATH=\$PYTHONPATH:\$CARLA_ROOT/PythonAPI/carla" >> ~/.bashrc
source ~/.bashrc
```

### 3.4 Workspace Build (ROS Bridge + Your Packages)

```bash
mkdir -p $PROOT/ros2_bridge_ws/src
cd       $PROOT/ros2_bridge_ws/src

# bridge
git clone -b 0.9.15 https://github.com/carla-simulator/ros-bridge.git

# your custom depth package (example)
git clone https://github.com/<you>/monocular_depth_pkg.git

cd $PROOT/ros2_bridge_ws
colcon build --symlink-install

# persist sourcing
echo "source $PROOT/ros2_bridge_ws/install/setup.bash" >> ~/.bashrc
```

### 3.5 Python Extras (optional but handy)

```bash
python3 -m pip install --user matplotlib pandas ultralytics==8.* tqdm
```

---

## Launch Sequence

| Terminal | Commands |
|----------|----------|
| T-1 – CARLA | `$CARLA_ROOT/CarlaUE4.sh -prefernvidia -quality-level=Low -carla-port=2000` |
| T-2 – (Re)build | `cd $PROOT/ros2_bridge_ws && colcon build` (skip if already built) |
| T-3 – Bridge + Ego | `source $PROOT/ros2_bridge_ws/install/setup.bash`<br>`ros2 launch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch.py` |
| T-4 – Your Node | `cd $PROOT/ros2_bridge_ws/src`<br>`ros2 run monocular_depth_pkg image_viewer` |

**Tip**: keep a small shell script `./scripts/startup.sh` that tmux-splits and fires all four commands.

---

## Verification Tests

- `ros2 topic list | wc -l` → ≈ 80 topics.
- `ros2 topic echo --once /carla/ego_vehicle/vehicle_status` prints steering/throttle.
- `ros2 bag record /carla/ego_vehicle/camera/rgb/image_color -o bags/day1 --duration 10` records a 10-s bag (~50 MB).
- `rqt_graph` shows something like:

![rqt_graph](docs/img/day1_rqt_graph.png)

---

## Day-1 Deliverables

| Artifact | Path / Branch | Status |
|----------|---------------|--------|
| 10-s rosbag | `bags/day1/` | ✅ |
| Setup scripts | `scripts/install_all.sh`, `scripts/startup.sh` | ✅ |
| Screenshots | `/docs/img/day1_rqt_graph.png`, `/docs/img/day1_topics.png` | 📸 |
| Git commit | `setup/env` merged to main | ✅ |
| Job-search links | Trello board "ADAS roles" column | 📋 |

**Milestone badge**: [Day-1 ✔] CARLA ↔ ROS2 bridge running • Sensor topics verified • Bag captured

---

## Troubleshooting

| Issue | Fix |
|-------|-----|
| `ros2 topic list` shows only `/clock` | Wait for CARLA to finish loading, then restart bridge; or port 2000 blocked. |
| Bridge crashes with `std::bad_alloc` | Add `-RenderOffScreen` and keep `-quality-level=Low`; make sure GPU driver matches CUDA runtime inside CARLA. |
| DDS domain clash | `export ROS_DOMAIN_ID=42` in every terminal (choose a free ID). |
| YOLO model not found | Download YOLOv8 models: `wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8n.pt` |

---

## References

- [ROS2 Humble Crash Course](https://youtu.be/Gg25GfA456o)
- [CARLA 0.9.15 Docs – Running an Instance](https://carla.readthedocs.io/en/0.9.15/running_carla/)
- [ros-bridge 0.9.15 README](https://github.com/carla-simulator/ros-bridge/tree/0.9.15)

---

## Next Up → Day 2

### Record ROS2 bag
'''ros2 bag record /carla/hero/rgb_front/image \
                /carla/hero/rgb_front/camera_info \
                /carla/hero/odometry \
                /carla/hero/imu \
                /tf \
                /clock
'''
---

### Why this README is "complete"

* **Turn-key reproducible** – anyone can follow it on a fresh VM.  
* **Verification tests** – prove the setup works.  
* **Day-1 deliverables logged** – clear checkpoint for mentors/recruiters.  
* **Troubleshooting** – lowers issue churn for collaborators.

Drop the markdown into your repo, add your screenshots, `git add README.md`, and you're done with Day 1 documentation. Happy hacking! 



Record