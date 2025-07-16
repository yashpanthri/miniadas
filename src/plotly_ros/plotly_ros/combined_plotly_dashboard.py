#!/usr/bin/env python3

'''
CARLA Dashboard + Camera Viewer

Dashboard visualizations:
1. X Position vs. Time
2. Y Position vs. Time
3. Y Position vs. X position
4. GNSS Longitude/Latitude vs. Time
5. GNSS Longitude vs. Latitude
6. IMU Acceleration (X, Y, Z) vs. Time
7. Speed vs. Time
8. Camera Frame

ROS2 humble . Python 3.10 . Dash 3.1.1
'''


# std-lib & third-party
from collections import deque # For Storing points
from threading import Thread # For executing ROS and dash together
from math import atan2 # Calculates tan inverse
import base64 # Converts incoming camera message to stream of data used by HTML and dash

# ROS2 & msgs
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, Imu, Image
from std_msgs.msg import Float32
from rclpy.time import Time

# Image conversion libraries
from cv_bridge import CvBridge
import cv2

# Dash and plotly
import dash
from dash import dcc, html
import plotly.graph_objs as go
global tstamp

def quaternion_to_yaw(q):
    '''
    This method converts geometry_msgs/Quaternion --> yaw(rad)
    '''
    siny_cosp = 2.0 * (q.w * q.z + q.x *q.y)
    cosy_cosp = 1.0 -2.0*(q.y * q.y + q.z * q.z)
    return atan2(siny_cosp, cosy_cosp)



class CarlaDataBuffer(Node):
    '''
    Subscribe to the topics we want to plot & keeps recent data only.
    '''
    BUFFER_LEN = 2_000 # ≈40 s @50 Hz – adjust for your use‑case


    def __init__(self):
        super().__init__("Carla_data_buffer")
        self.bridge = CvBridge() # for Image --> cv2 --> PNG

        # --------- Deques for ----------------
        self.odometry_txy = deque(maxlen = self.BUFFER_LEN) # Stores position (x,y)
        self.gnss_tlat_lon = deque(maxlen = self.BUFFER_LEN) # Stores latitude and longitude (lat, lon)
        self.imu_taccel = deque(maxlen = self.BUFFER_LEN) # Stores accleration data from imu sensor (ax, ay, az)
        self.speed_tmps = deque(maxlen = self.BUFFER_LEN) # Stores speedometer values of datatype Float32
        self.camera_frames = None # Stores latest PNG base64 format

        # --------- Subscriptions ----------------
        #1. Odometry
        self.create_subscription(Odometry, '/carla/hero/odometry', self.odom_callback, 20)
        
        #2. GNSS
        self.create_subscription(NavSatFix, '/carla/hero/gnss', self.gnss_callback, 20)

        #3. IMU
        self.create_subscription(Imu, '/carla/hero/imu', self.imu_callback, 50)

        #4. Speedometer (Float32 (m/s))
        self.create_subscription(Float32, '/carla/hero/speedometer', self.speed_callback, 20)

        #5. Front RGB Camera
        self.create_subscription(Image, '/carla/hero/egb_front_image', self.odom_callback, 20)

        # #1. Odometry
        # self.create_subscription(Odometry, '/carla/hero/odometry', self.odom_callback, 20)



        self.get_logger().info("Subscribed to Odometry, GNSS, IMU, Speedometer and Front RGB Camera")
    
    
    # --------- Timestamp Methods ----------------
    def timestamp(self, stamp):
        '''Return header stamp as float seconds (3 decimal places).'''
        return round(stamp.sec + stamp.nanosec * 1e-9, 3)
    

#-------------------------------------------------------------------------------------------------------
    # --------- Callback Methods -----------------------------------------------------------------------
#-------------------------------------------------------------------------------------------------------

    '''
    /carla/hero/odometry (topic) —  nav_msgs/msg/Odometry (msg)
    std_msgs/Header header {stamp, frame_id}
    string child_frame_id
    geometry_msgs/PoseWithCovariance pose
    geometry_msgs/TwistWithCovariance twist
    '''
    def odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        t = self.timestamp(msg.header.stamp)
        self.odometry_txy.append((t, p.x, p.y))


    '''
    /carla/hero/gnss (topic) —  sensor_msgs/msg/NavSatFix (msg)
    std_msgs/Header header {stamp, frame_id}
    float64 latitude, longitude, altitude
    sensor_msgs/NavSatStatus status
    '''
    def gnss_callback(self, msg: NavSatFix):
        t = self.timestamp(msg.header.stamp)
        self.gnss_tlat_lon.append((t, msg.latitude, msg.longitude))


    '''
    /carla/hero/imu (topic) —  sensor_msgs/msg/Imu (msg)
    std_msgs/Header header {stamp, frame_id}
    orientation (Quaternion)
    angular_velocity (Vector3)
    linear_acceleration (Vector3)
    '''
    def imu_callback(self, msg: Imu):
        a = msg.linear_acceleration
        t = self.timestamp(msg.header.stamp)
        self.imu_taccel.append((t, a.x, a.y, a.z))


    '''
    /carla/hero/speedometer (topic) —  std_msgs/msg/Float32 (msg)
    float32 data  # vehicle speed (m/s)
    *Publisher may omit Header; we timestamp locally*
    '''
    def speed_callback(self, msg: Float32):
        t = round(self.get_clock().now().nanoseconds * 1e-9, 3)
        self.speed_tmps.append((t, msg.data))


    '''
    /carla/hero/rgb_front/image (topic) —  sensor_msgs/msg/Image (msg)
    uint32 height, width, step
    string encoding, uint8 is_bigendian
    uint8[] data (row-major pixels)
    '''
    def cam_callback(self, msg: Image):
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv_img = cv2.resize(cv_img, (640, 360)) # Resize for faster UI refresh
        _, png = cv2.imencode('.png', cv_img)
        self.camera_frames = base64.b64encode(png.tobytes()).decode('utf-8')



# ========================= Dash app =================================================

def launch_dash(node: CarlaDataBuffer):
    '''
    Run Plotly Dash server in a "daemon thread" so ROS spin() remains main.
    '''

    # -------------- Dash initialisation ---------------------------------------------
    app = dash.Dash(__name__)

    # -------------- Layout (single column) ------------------------------------------
    app.layout = html.Div(
        [
            html.H2("CARLA Sensor Dashboard"),
            dcc.Graph(id="odomxt-plot"),
            dcc.Graph(id="odomyt-plot"),
            dcc.Graph(id="odomxy-plot"),
            dcc.Graph(id="gnsslonlatt-plot"),
            dcc.Graph(id="gnsslonlat-plot"),
            dcc.Graph(id="imuxyzt-plot"),
            dcc.Graph(id="speed-plot"),
            html.Img(id="cam-img", style={"width": "100%"}),
            # Tick every 500 ms
            dcc.Interval(id="tick", interval=500, n_intervals=0),
        ],
        style={"width": "80%", "margin": "0 auto"},
    )

    # ----------------- Callback methods for Dashboard -------------------------------
    
    @app.callback(
        dash.Output("odomxt-plot", "figure"),
        dash.Input("tick", "n_intervals"),
    )
    def _update_odom(_):
        if not node.odometry_txy:
            return go.Figure().update_layout(title="Waiting for x position data…")
        ts, xs, _ = zip(*node.odometry_txy)
        fig = go.Figure(
            go.Scatter(x=ts, y=xs, mode="lines+markers", marker={"size": 4})
        )
        fig.update_layout(
            title=f"Ego-vehicle X Position vs. Time  •  {len(xs)} pts",
            xaxis_title="Time [s]",
            yaxis_title="X Position [m]",
        )
        fig.update_yaxes(scaleanchor="x", scaleratio=1)  # 1:1 aspect
        return fig
    

    @app.callback(
        dash.Output("odomyt-plot", "figure"),
        dash.Input("tick", "n_intervals"),
    )
    def _update_odom(_):
        if not node.odometry_txy:
            return go.Figure().update_layout(title="Waiting for Y position data…")
        ts, _, ys = zip(*node.odometry_txy)
        fig = go.Figure(
            go.Scatter(x=ts, y=ys, mode="lines+markers", marker={"size": 4})
        )
        fig.update_layout(
            title=f"Ego-vehicle Y Position vs. Time  •  {len(ys)} pts",
            xaxis_title="Time [s]",
            yaxis_title="Y Position [m]",
        )
        fig.update_yaxes(scaleanchor="x", scaleratio=1)  # 1:1 aspect
        return fig
    
    
    @app.callback(
        dash.Output("odomxy-plot", "figure"),
        dash.Input("tick", "n_intervals"),
    )
    def _update_odom(_):
        if not node.odometry_txy:
            return go.Figure().update_layout(title="Waiting for X and Y Position odometry data…")
        _, xs, ys = zip(*node.odometry_txy)
        fig = go.Figure(
            go.Scatter(x=xs, y=ys, mode="lines+markers", marker={"size": 4})
        )
        fig.update_layout(
            title=f"Ego-vehicle X vs Y Position  •  {len(xs)} pts",
            xaxis_title="X Position [m]",
            yaxis_title="Y Position [m]",
        )
        fig.update_yaxes(scaleanchor="x", scaleratio=1)  # 1:1 aspect
        return fig

    
    @app.callback(
        dash.Output("gnsslonlatt-plot", "figure"),
        dash.Input("tick", "n_intervals"),
    )
    def _update_gnss(_):
        if not node.gnss_tlat_lon:
            return go.Figure().update_layout(title="Waiting for GNSS Longitude/Latitude vs Time…")
        ts, lats, lons = zip(*node.gnss_tlat_lon)
        fig = go.Figure(
            [
            go.Scatter(x=ts, y=lons, mode="markers", marker={"size": 6}),
            go.Scatter(x=ts, y=lats, mode="markers", marker={"size": 6})
            ]
        )
        fig.update_layout(
            title="GNSS: Longitude/Latitude vs. Time",
            xaxis_title="Time [s]",
            yaxis_title="Longitude/Latitude [°]",
        )
        return fig


    @app.callback(
        dash.Output("gnsslonlat-plot", "figure"),
        dash.Input("tick", "n_intervals"),
    )
    def _update_gnss(_):
        if not node.gnss_tlat_lon:
            return go.Figure().update_layout(title="Waiting for GNSS Latitude vs GNSS Longitude…")
        _, lats, lons = zip(*node.gnss_tlat_lon)
        fig = go.Figure(
            go.Scatter(x=lons, y=lats, mode="markers", marker={"size": 6})
        )
        fig.update_layout(
            title="GNSS: Latitude vs Longitude",
            xaxis_title="Longitude [°]",
            yaxis_title="Latitude [°]",
        )
        return fig


    @app.callback(
        dash.Output("imuxyzt-plot", "figure"),
        dash.Input("tick", "n_intervals"),
    )
    def _update_imu(_):
        if not node.imu_taccel:
            return go.Figure().update_layout(title="Waiting for IMU acceleration(X,Y,Z) vs Time…")
        ts, ax, ay, az = zip(*node.imu_taccel)
        fig = go.Figure([
                go.Scatter(x=ts, y=ax, mode="lines", name="ax"),
                go.Scatter(x=ts, y=ay, mode="lines", name="ay"),
                go.Scatter(x=ts, y=az, mode="lines", name="az"),
            ])
        fig.update_layout(
            title="IMU linear acceleration",
            xaxis_title="Time [s]",
            yaxis_title="Acceleration [m/s²]",
        )
        return fig


    @app.callback(
        dash.Output("speed-plot", "figure"),
        dash.Input("tick", "n_intervals"),
    )
    def _update_speed(_):
        if not node.speed_tmps:
            return go.Figure().update_layout(title="Waiting for speedometer…")
        ts, speeds = zip(*node.speed_tmps)
        fig = go.Figure(go.Scatter(x = ts, y=speeds, mode="lines"))
        fig.update_layout(
            title="Vehicle speed",
            xaxis_title="Time [s]",
            yaxis_title="Speed [m/s]",
        )
        return fig


    @app.callback(
        dash.Output("cam-img", "src"),
        dash.Input("tick", "n_intervals"),
    )
    def _update_cam(_):
        if node.camera_frames is None:
            return ""  # Blank until the first frame arrives
        return f"data:image/png;base64, {node.camera_frames}"


    # ---- Run Dash App ----------------------------------------------------------
    # Dash ≥3 prefers app.run() (run_server still works but warns)
    app.run(host="0.0.0.0", port=8051, debug=False, use_reloader=False)







# --------------------------------------------------------------------------------
# --------------------------------- main -----------------------------------------
# --------------------------------------------------------------------------------


def main():
    '''
    Initialise ROS, then start Dash in a daemon thread.
    '''
    
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

