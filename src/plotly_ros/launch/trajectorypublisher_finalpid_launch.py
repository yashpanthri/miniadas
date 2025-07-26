from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()    

    t1 = Node(
        package = "plotly_ros",
        executable = 'trajectory_publisher',
        # remappings = [
        #     ("turtle1/cmd_vel", "speed")
        # ]

    )
    t2 = Node(
        package = "plotly_ros",
        executable = 'final_pid',
        # remappings = [
        #     ("turtle1/cmd_vel", "speed")
        # ],
        parameters = [
            {"ctrl_hz": 20.0},
            {"kp_pos": 0.05},
            {"kp_vel": 6.0},
            {"ki": 0.5},
            {"kd": 0.5},
            {"lookahead_distance": 8.0},  # Reduced from 15.0
            # Pure pursuit steering parameters
            {"wheelbase": 2.875},
            {"k0": 3.0},  # Reduced from 5.0 - smaller base lookahead
            {"kv": 0.3},  # Reduced from 0.5 - less speed-dependent scaling
            {"max_steering_angle": 0.61}
        ]
    )
    #ld = LaunchDescription([t1, t2])
    ld.add_action(t1)
    ld.add_action(t2)
    return ld