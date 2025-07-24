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
        # parameters = [
        #     {"linear_velocity": 5.0},
        #     {"angular_angular": 2.0}
        # ]
    )
    #ld = LaunchDescription([t1, t2])
    ld.add_action(t1)
    ld.add_action(t2)
    return ld