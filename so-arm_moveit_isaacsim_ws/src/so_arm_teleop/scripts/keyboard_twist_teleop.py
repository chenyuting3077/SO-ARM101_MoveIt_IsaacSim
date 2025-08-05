#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import sys
import termios
import tty
import select

msg = """
控制機械臂末端（End Effector）:

移動：
  W/S : 前 / 後 (X軸)
  A/D : 左 / 右 (Y軸)
  Q/E : 上 / 下 (Z軸)

旋轉：
  I/K : Pitch ↑↓
  J/L : Yaw ←→
  U/O : Roll ↺↻

空白鍵：停止（全部歸 0）
Q: 離開
"""

# 每次按鍵對應的線速度/角速度
LIN_STEP = 0.1  # m/s
ANG_STEP = 0.5  # rad/s

def get_key():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.2)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class TwistTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_twist_teleop')
        self.publisher_ = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.twist = TwistStamped()
        self.twist.header.frame_id = 'base'  # 與 servo.yaml 中 planning_frame 一致
        self.print_instructions()

    def print_instructions(self):
        print(msg)

    def send_twist(self):
        self.twist.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.twist)

def main(args=None):
    global settings
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init(args=args)
    node = TwistTeleop()

    try:
        while True:
            key = get_key()
            twist = node.twist.twist

            # 歸零
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0

            if key == 'w':
                twist.linear.x = LIN_STEP
            elif key == 's':
                twist.linear.x = -LIN_STEP
            elif key == 'a':
                twist.linear.y = LIN_STEP
            elif key == 'd':
                twist.linear.y = -LIN_STEP
            elif key == 'q':
                twist.linear.z = LIN_STEP
            elif key == 'e':
                twist.linear.z = -LIN_STEP
            elif key == 'i':
                twist.angular.y = ANG_STEP
            elif key == 'k':
                twist.angular.y = -ANG_STEP
            elif key == 'j':
                twist.angular.z = ANG_STEP
            elif key == 'l':
                twist.angular.z = -ANG_STEP
            elif key == 'u':
                twist.angular.x = ANG_STEP
            elif key == 'o':
                twist.angular.x = -ANG_STEP
            elif key == ' ':
                print("🔁 歸零")
            elif key == 'Q' or key == 'q':
                print("❌ 離開")
                break
            else:
                continue

            node.send_twist()

    except Exception as e:
        print(f"錯誤：{e}")

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
