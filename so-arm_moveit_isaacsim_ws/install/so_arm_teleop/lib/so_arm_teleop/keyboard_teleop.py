#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import sys, select, termios, tty

instructions = """
WASD 控制機械手臂末端執行器 (EE)
================================
    w     ↑ (+X)
  a s d ← ↓ → (+Y/-Y, -X)
    x     ↓ (-Z)
    q     ↑ (+Z)

姿態控制：
  u/o：Roll 左/右
  h/n：Pitch 上/下
  y/m：Yaw 左/右

空白鍵：停止
CTRL-C：離開
"""

move_bindings = {
    'w': (1, 0, 0, 0, 0, 0),
    's': (-1, 0, 0, 0, 0, 0),
    'a': (0, 1, 0, 0, 0, 0),
    'd': (0, -1, 0, 0, 0, 0),
    'q': (0, 0, 1, 0, 0, 0),
    'x': (0, 0, -1, 0, 0, 0),
    'u': (0, 0, 0, 1, 0, 0),
    'o': (0, 0, 0, -1, 0, 0),
    'h': (0, 0, 0, 0, 1, 0),
    'n': (0, 0, 0, 0, -1, 0),
    'y': (0, 0, 0, 0, 0, 1),
    'm': (0, 0, 0, 0, 0, -1),
}

def get_key():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

settings = termios.tcgetattr(sys.stdin)

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.publisher = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.linear_speed = 0.1
        self.angular_speed = 0.5
        self.twist = TwistStamped()

    def publish_twist(self):
        self.twist.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.twist)

    def spin(self):
        print(instructions)
        try:
            while True:
                key = get_key()
                if key == '\x03':
                    break
                elif key == ' ':
                    self.twist.twist = TwistStamped().twist
                    self.publish_twist()
                elif key in move_bindings:
                    x, y, z, rx, ry, rz = move_bindings[key]
                    self.twist.twist.linear.x = x * self.linear_speed
                    self.twist.twist.linear.y = y * self.linear_speed
                    self.twist.twist.linear.z = z * self.linear_speed
                    self.twist.twist.angular.x = rx * self.angular_speed
                    self.twist.twist.angular.y = ry * self.angular_speed
                    self.twist.twist.angular.z = rz * self.angular_speed
                    self.publish_twist()
        finally:
            self.twist.twist = TwistStamped().twist
            self.publish_twist()

def main():
    rclpy.init()
    teleop = KeyboardTeleop()
    teleop.spin()
    teleop.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
