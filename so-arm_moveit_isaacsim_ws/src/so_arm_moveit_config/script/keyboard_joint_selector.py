#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys, select, termios, tty

# 關節列表
joint_names = [
    "Rotation", "Pitch", "Elbow", "Wrist_Pitch", "Wrist_Roll", "Jaw"
]

# 每次按鍵改變角度量（弧度）
joint_step = 0.05

help_msg = """
控制說明：

數字鍵 1~6：選擇要控制的關節
  1: Rotation
  2: Pitch
  3: Elbow
  4: Wrist_Pitch
  5: Wrist_Roll
  6: Jaw

+ / ↑：增加角度
- / ↓：減少角度
q：退出
"""

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.2)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class JointTeleop(Node):
    def __init__(self):
        super().__init__('joint_selector_teleop')
        self.publisher_ = self.create_publisher(JointTrajectory, '/issac_joint_command', 10)
        self.joint_positions = {name: 0.0 for name in joint_names}
        self.current_joint_index = 0  # 預設選第一個 joint

    def print_status(self):
        print("\n目前關節：{} ({})".format(
            self.current_joint_index + 1, joint_names[self.current_joint_index]
        ))
        for i, name in enumerate(joint_names):
            marker = "👉" if i == self.current_joint_index else "  "
            print(f"{marker} {i+1}. {name}: {self.joint_positions[name]:.3f} rad")

    def update_joint(self, delta):
        name = joint_names[self.current_joint_index]
        self.joint_positions[name] += delta
        self.publish_command()

    def select_joint(self, index):
        if 0 <= index < len(joint_names):
            self.current_joint_index = index
            self.print_status()

    def publish_command(self):
        traj = JointTrajectory()
        traj.joint_names = joint_names
        point = JointTrajectoryPoint()
        point.positions = [self.joint_positions[name] for name in joint_names]
        point.time_from_start.sec = 1
        traj.points = [point]
        self.publisher_.publish(traj)

def main(args=None):
    global settings
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init(args=args)
    node = JointTeleop()

    print(help_msg)
    node.print_status()

    try:
        while True:
            key = getKey()
            if key == 'q':
                break
            elif key in ['1', '2', '3', '4', '5', '6']:
                node.select_joint(int(key) - 1)
            elif key in ['+', '=', 'w', 'W', '\x1b[A']:  # ↑
                node.update_joint(+joint_step)
                node.print_status()
            elif key in ['-', '_', 's', 'S', '\x1b[B']:  # ↓
                node.update_joint(-joint_step)
                node.print_status()
    except Exception as e:
        print(f"例外：{e}")
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
