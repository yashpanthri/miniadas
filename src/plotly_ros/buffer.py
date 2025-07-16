#!/usr/bin/env python3
'''
Comprehensive Plotly Dash dashboard for multiple CARLA ROS 2 topics
------------------------------------------------------------------
This version **adds millisecond‑precision timestamps** (rounded to three
decimal places) to every odometry sample.  Each deque entry is now a
triple `(t, x, y)` where `t` is seconds since epoch (or ROS clock) with
3‑decimal precision (e.g. `42.137`).  The layout is still a single page
(without tabs).
'''

# ------------------------- standard libs -----------------------------------
from collections import deque
from math import atan2
from threading import Thread
import base64           # camera frame → base64 PNG for Dash

# ------------------------- ROS 2 / msgs ------------------------------------
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, Imu, Image
from std_msgs.msg import Float32  # speedometer publishes a Float64 in CARLA

# cv_bridge lets us turn ROS Image → OpenCV → PNG bytes
from cv_bridge import CvBridge
import cv2

# ------------------------- Dash & Plotly -----------------------------------
import dash
from dash import dcc, html
import plotly.graph_objs as go


# ========================= helper functions ================================

def quaternion_to_yaw(q):
    """Convert *geometry_msgs/Quaternion* → yaw (rad)."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return atan2(siny_cosp, cosy_cosp)


# ========================= ROS 2 node ======================================

class CarlaDataBuffer(Node):
    """Subscribes to the topics we want to plot & keeps *recent* data only."""

    BUFFER_LEN = 2_000  # ≈40 s @50 Hz – adjust for your use‑case

    def __init__(self):
        super().__init__('carla_data_buffer')
        self.bridge = CvBridge()  # for Image → cv2 → PNG

        # ---------------- circular buffers ---------------------------------
        self.odometry_xy   = deque(maxlen=self.BUFFER_LEN)   # (t, x, y)
        self.gnss_latlon   = deque(maxlen=self.BUFFER_LEN)   # (lat, lon)
        self.imu_acceleration     = deque(maxlen=self.BUFFER_LEN)   # (ax, ay, az)
        self.speed_mps     = deque(maxlen=self.BUFFER_LEN)   # float
        self.camera_frame  = None                            # latest PNG b64

        # ---------------- subscriptions ------------------------------------
        # ① Odometry
        self.create_subscription(Odometry, '/carla/hero/odometry',
                                 self.odom_callback, 20)
        # ② GNSS
        self.create_subscription(NavSatFix, '/carla/hero/gnss',
                                 self.gnss_callback, 20)
        # ③ IMU
        self.create_subscription(Imu, '/carla/hero/imu',
                                 self.imu_callback, 50)
        # ④ Speedometer (float64 [m/s])
        self.create_subscription(Float32, '/carla/hero/speedometer',
                                 self.speed_callback, 20)
        # ⑤ Front‑facing RGB camera
        self.create_subscription(Image,
                                 '/carla/hero/rgb_front/image', # /carla/hero/front_camera/image_raw
                                 self.cam_callback, 20)

        self.get_logger().info('Subscribed to odometry, GNSS, IMU, speedometer, camera')

    # ---------------- callback methods -------------------------------------
    def odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        # Build time stamp (secs + nsecs) and round to 3 decimals (millisecond)
        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        t = round(stamp_sec, 3)
        self.odometry_xy.append((t, p.x, p.y))

    def gnss_callback(self, msg: NavSatFix):
        self.gnss_latlon.append((msg.latitude, msg.longitude))

    def imu_callback(self, msg: Imu):
        a = msg.linear_acceleration
        self.imu_acceleration.append((a.x, a.y, a.z))

    def speed_callback(self, msg: Float32):
        self.speed_mps.append(msg.data)

    def cam_callback(self, msg: Image):
        """Convert ROS image to base64‑encoded PNG for Dash."""
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # Resize for faster UI refresh (optional)
        cv_img = cv2.resize(cv_img, (640, 360))
        _, png = cv2.imencode('.png', cv_img)
        self.camera_frame = base64.b64encode(png.tobytes()).decode('utf-8')  #'ascii'


# ========================= Dash app ========================================

def launch_dash(node: CarlaDataBuffer):
    """Run Plotly Dash server in a **daemon thread** so ROS spin() remains main."""

    app = dash.Dash(__name__)

    # ---------- layout ------------------------------------------------------
    app.layout = html.Div([
        html.H2("CARLA Sensor Dashboard"),
        # Odometry path
        dcc.Graph(id='odom-plot'),
        # GNSS scatter
        dcc.Graph(id='gnss-plot'),
        # IMU
        dcc.Graph(id='imu-plot'),
        # Speed
        dcc.Graph(id='speed-plot'),
        # Camera feed
        html.Img(id='cam-img', style={'width': '100%'}),
        # Master timer – every 0.5 s ask callbacks to regenerate figures
        dcc.Interval(id='tick', interval=500, n_intervals=0),
    ], style={'width': '80%', 'margin': '0 auto'})

    # ---------- callbacks ---------------------------------------------------
    # 1️⃣ Odometry path
    @app.callback(dash.Output('odom-plot', 'figure'), dash.Input('tick', 'n_intervals'))
    def _update_odom(_):
        if not node.odometry_xy:
            return go.Figure().update_layout(title='Waiting for odometry…')
        ts, xs, ys = zip(*node.odometry_xy)  # unpack triple, ignore ts for path
        fig = go.Figure(go.Scatter(x=xs, y=ys, mode='lines+markers', marker={'size': 4}))
        fig.update_layout(title=f'Ego-vehicle path  •  {len(xs)} pts',
                          xaxis_title='x [m]', yaxis_title='y [m]')
        fig.update_yaxes(scaleanchor='x', scaleratio=1)  # keep aspect = 1
        return fig

    # 2️⃣ GNSS scatter
    @app.callback(dash.Output('gnss-plot', 'figure'), dash.Input('tick', 'n_intervals'))
    def _update_gnss(_):
        if not node.gnss_latlon:
            return go.Figure().update_layout(title='Waiting for GNSS…')
        lat, lon = zip(*node.gnss_latlon)
        fig = go.Figure(go.Scatter(x=lon, y=lat, mode='markers', marker={'size': 6}))
        fig.update_layout(title='GNSS fix', xaxis_title='Longitude [°]', yaxis_title='Latitude [°]')
        return fig

    # 3️⃣ IMU accel time‑series
    @app.callback(dash.Output('imu-plot', 'figure'), dash.Input('tick', 'n_intervals'))
    def _update_imu(_):
        if not node.imu_acceleration:
            return go.Figure().update_layout(title='Waiting for IMU…')
        ax, ay, az = zip(*node.imu_acceleration)
        fig = go.Figure([
            go.Scatter(y=ax, mode='lines', name='ax'),
            go.Scatter(y=ay, mode='lines', name='ay'),
            go.Scatter(y=az, mode='lines', name='az'),
        ])
        fig.update_layout(title='IMU linear acceleration', xaxis_title='Sample index', yaxis_title='m/s²')
        return fig

    # 4️⃣ Speedometer time‑series
    @app.callback(dash.Output('speed-plot', 'figure'), dash.Input('tick', 'n_intervals'))
    def _update_speed(_):
        if not node.speed_mps:
            return go.Figure().update_layout(title='Waiting for speedometer…')
        fig = go.Figure(go.Scatter(y=list(node.speed_mps), mode='lines'))
        fig.update_layout(title='Vehicle speed', xaxis_title='Sample index', yaxis_title='m/s')
        return fig

    # 5️⃣ Camera frame (returns **src** attribute string)
    @app.callback(dash.Output('cam-img', 'src'), dash.Input('tick', 'n_intervals'))
    def _update_cam(_):
        if node.camera_frame is None:
            return ''  # empty until first frame arrives
        return f'data:image/png;base64, {node.camera_frame}'

    # ---------- start Dash --------------------------------------------------
    app.run(host='0.0.0.0', port=8050, debug=False, use_reloader=False)


# ========================= main ============================================

def main():
    """Initialise ROS, then start Dash in a daemon thread."""
    rclpy.init()
    node = CarlaDataBuffer()

    # Dash runs in the background; ROS keeps the main thread.
    Thread(target=launch_dash, args=(node,), daemon=True).start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
