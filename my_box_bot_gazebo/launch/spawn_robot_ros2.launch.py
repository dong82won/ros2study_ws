from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    urdf_visualize_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('my_box_bot_description'),
                'launch',
                'urdf_visualize_meshes_collisions_inertias.launch.py'
            ])
        ])
    )

    spawn_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('my_box_bot_gazebo'),
                'launch',
                'spawn_robot_description.launch.py'
            ])
        ])
    )

    return LaunchDescription([
        urdf_visualize_launch,
        spawn_robot_launch
    ])