#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateRepublisher(Node):
    def __init__(self):
        super().__init__('joint_state_republisher')

        self.target_joint_names = [
            'Rotation', 'Pitch', 'Elbow', 'Wrist_Pitch', 'Wrist_Roll', 'Jaw'
        ]

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',  # From MoveIt / Servo
            self.listener_callback,
            10)

        self.publisher = self.create_publisher(JointState, '/isaac_joint_command', 10)

    def listener_callback(self, msg):
        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()

        # 只轉你關心的關節
        out.name = []
        out.position = []
        for name in self.target_joint_names:
            if name in msg.name:
                idx = msg.name.index(name)
                out.name.append(name)
                out.position.append(msg.position[idx])

        self.publisher.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = JointStateRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
