from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('so_arm_moveit_config'),
        'config',
        'servo.yaml'
    )

    return LaunchDescription([
        # 啟動 MoveIt Servo Node
        Node(
            package='moveit_servo',
            executable='servo_node_main',
            name='servo_node',
            output='screen',
            parameters=[config]
        ),

        # 啟動鍵盤控制 Node
        Node(
            package='so_arm_moveit_config',
            executable='keyboard_teleop.py',
            name='keyboard_teleop',
            output='screen'
        )
    ])
