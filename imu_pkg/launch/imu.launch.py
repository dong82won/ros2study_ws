import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    launch_description = LaunchDescription()

    # Get the path to the package's share directory
    share_dir = get_package_share_directory('imu_pkg')
    param_file_name = 'imu_config.yaml'
    param_file_path = os.path.join(share_dir, 'config', param_file_name)

    imu_node = Node(
        package='imu_pkg',
        executable='imu_node',
        name='imu_pkg',
        output='screen',
        parameters=[param_file_path],
    )

    # Add Nodes to the launch description
    launch_description.add_action(imu_node)

    return launch_description
