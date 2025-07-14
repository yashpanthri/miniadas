#!/usr/bin/env python3
"""
Live 2-D trajectory viewer for CARLA ego vehicle.
Subscribes to /carla/hero/odometry and streams the (x, y) path into a Plotly Dash
scatter plot that refreshes twice per second.

Tested on ROS 2 Humble / Python 3.10 / Dash 3.x.
"""

from collections import deque 
from math import atan2
from threading import Thread

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

import dash
from dash import dcc, html
import plotly.graph_objs as go


# ---------- helper -----------------------------------------------------------

def quaternion_to_yaw(q):
    """geometry_msgs/Quaternion ➜ yaw (Z-axis rotation) in radians."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return atan2(siny_cosp, cosy_cosp)


# ---------- ROS 2 node -------------------------------------------------------

class OdomBuffer(Node):
    """Maintains a rolling buffer of recent XY positions and yaw angles."""
    BUFFER_LEN = 2_000     # store this many trajectory points

    def __init__(self):
        super().__init__('odom_buffer')
        self.xy  = deque(maxlen=self.BUFFER_LEN)   # (x, y)
        self.yaw = deque(maxlen=self.BUFFER_LEN)   # optional

        self.create_subscription(
            Odometry,
            '/carla/hero/odometry',
            self.odom_callback,
            20,             # QoS queue depth
        )
        self.get_logger().info('Subscribed to /carla/hero/odometry')

    def odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.xy.append((p.x, p.y))
        self.yaw.append(quaternion_to_yaw(q))


# ---------- Dash app ---------------------------------------------------------

def launch_dash_odometry(node: OdomBuffer):
    """Run the Dash server in its own thread so ROS can spin."""
    app = dash.Dash(__name__)

    app.layout = html.Div(
        [
            html.H3('CARLA Ego-Vehicle Ideal/Ground Truth Trajectory'),
            dcc.Graph(id='traj-plot'),
            dcc.Interval(id='tick', interval=500, n_intervals=0),   # 500 ms
        ],
        style={'width': '60%', 'margin': '0 auto'},
    )

    @app.callback(
        dash.Output('traj-plot', 'figure'),
        [dash.Input('tick', 'n_intervals')],
    )
    def update_plot(_):
        """Rebuild scatter every 500 ms."""
        if not node.xy:
            return go.Figure().update_layout(
                xaxis_title='x [m]', yaxis_title='y [m]',
                title='Waiting for data…'
            ) # Sending empty plotly figure if node.xy is empty 

        xs, ys = zip(*node.xy) #xs --> [x1,x2,x3...] #ys --> [y1,y2,y3...] #that is unpacks data
        fig = go.Figure(go.Scatter(
            x=xs, y=ys,
            mode='lines+markers',
            marker=dict(size=4),
            name='trajectory',
        ))
        
        fig.update_layout(
            xaxis_title='x [m]', yaxis_title='y [m]',
            title=f'2-D path ({len(xs)} points)',
            autosize=True,
            margin=dict(l=40, r=20, t=50, b=40),
        )
        fig.update_yaxes(scaleanchor='x', scaleratio=1)  # square aspect
        return fig

    # Dash ≥ 3: use app.run()
    app.run(host='0.0.0.0', port=8050, debug=False, use_reloader=False)


# ---------- main -------------------------------------------------------------

def main():
    rclpy.init()
    node = OdomBuffer()

    # Start Dash in a daemon thread so Ctrl-C stops everything cleanly
    Thread(target=launch_dash_odometry, args=(node,), daemon=True).start()
    ''' Decomposition of above Syntax:
    Thread(...)         --- You're creating a new Thread object
    target=launch_dash  --- This tells the thread: “run the launch_dash() function.”
    The launch_dash() function expects one argument (in use ROS node), so you pass it as a tuple.
    daemon=True         --- Without it, Dash might keep running even after you Ctrl-C the ROS process.
    .start()            --- This starts the thread immediately.
    '''

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

