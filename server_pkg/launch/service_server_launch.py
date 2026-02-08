# from launch import LaunchDescription
# from launch_ros.actions import Node

# def generate_launch_description():
#     return LaunchDescription([
#         Node(
#             package='server_pkg',
#             executable='service_server',
#             name='service_server',
#             output='screen',
#             parameters=[{'use_sim_time': True}],
#             # remappings=[
#             #     ('/cmd_vel', '/cmd_vel')
#             # ]
#         )
#     ])

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='server_pkg',
            executable='service_server',
            output='screen'),
    ])