import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('minimal_robot_gazebo')
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot.urdf')

    return LaunchDescription([
        # Launch Ignition Gazebo (Gazebo Fortress or newer)
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-r', '--verbose'],
            output='screen'
        ),

        # Spawn robot using ros_ign_gazebo's 'create'
        Node(
            package='ros_ign_gazebo',
            executable='create',
            arguments=[
                '-name', 'my_robot',
                '-file', urdf_file
            ],
            output='screen'
        )
    ])