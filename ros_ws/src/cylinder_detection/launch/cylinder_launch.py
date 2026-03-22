from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg = get_package_share_directory('cylinder_detection')
    rviz = os.path.join(pkg, 'rviz', 'cylinder.rviz')

    return LaunchDescription([
        Node(
            package='cylinder_detection',
            executable='cylinder_node',
            output='screen'
        ),
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz],
            output='screen'
        )
    ])