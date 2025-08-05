from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    teleop_share_dir = get_package_share_directory('so_arm_teleop')
    servo_yaml_path = os.path.join(teleop_share_dir, 'config', 'servo.yaml')

    return LaunchDescription([
        Node(
            package='moveit_servo',
            executable='servo_node_main',
            name='servo_node',
            parameters=[servo_yaml_path],
            output='screen'
        ),
        Node(
            package='so_arm_teleop',
            executable='joint_state_republisher.py',
            name='joint_state_republisher',
            output='screen'
        )
    ])
