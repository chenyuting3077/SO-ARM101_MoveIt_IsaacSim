#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import sys
import termios
import tty
import select

class JointKeyboardControl(Node):
    def __init__(self):
        super().__init__('joint_keyboard_control')
        self.publisher_ = self.create_publisher(JointState, '/isaac_joint_command', 10)

        # 關節名稱（需與 /joint_states 中一致）
        self.joint_names = [
            'Rotation',
            'Pitch',
            'Elbow',
            'Wrist_Pitch',
            'Wrist_Roll',
            'Jaw'
        ]

        # 各關節目前的位置（初始化為 0）
        self.joint_positions = [0.0 for _ in self.joint_names]

        # 當前選中的關節索引（預設為第 1 個）
        self.selected_joint = 0

        # 每次按鍵操作所調整的關節角度（單位：度 or 弧度依需求）
        self.step_size = 0.01

        # 儲存原本的終端設定（用於結束時恢復）
        self.old_settings = termios.tcgetattr(sys.stdin)

        # 顯示控制說明
        self.print_instructions()

    def print_instructions(self):
        print("\n=== 機械手臂鍵盤控制 ===")
        print("1-6：選擇要控制的關節")
        print("a/d：調整選中關節的角度（a=減小，d=增加）")
        print("q：退出控制程式")
        print(f"目前選擇的關節：{self.joint_names[self.selected_joint]}")
        print(f"目前角度值：{self.joint_positions[self.selected_joint]:.4f}")

    def get_key(self):
        # 非阻塞讀取鍵盤輸入
        if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
            return sys.stdin.read(1)
        return None

    def run(self):
        try:
            # 設定終端為非阻塞模式
            tty.setcbreak(sys.stdin.fileno())

            while rclpy.ok():
                key = self.get_key()
                if key is not None:
                    # 選擇關節（1~6）
                    if key in ['1', '2', '3', '4', '5', '6']:
                        self.selected_joint = int(key) - 1
                        print(f"\n目前選擇的關節：{self.joint_names[self.selected_joint]}")
                        print(f"目前角度值：{self.joint_positions[self.selected_joint]:.4f}")

                    # 減少角度（a）
                    elif key == 'a':
                        self.joint_positions[self.selected_joint] -= self.step_size
                        self.publish_joint_command()
                        print(f"角度減少後：{self.joint_positions[self.selected_joint]:.4f}")

                    # 增加角度（d）
                    elif key == 'd':
                        self.joint_positions[self.selected_joint] += self.step_size
                        self.publish_joint_command()
                        print(f"角度增加後：{self.joint_positions[self.selected_joint]:.4f}")

                    # 退出程式
                    elif key == 'q':
                        print("\n已退出機械手臂控制")
                        break

                # 短暫休眠，避免佔用過高 CPU
                rclpy.spin_once(self, timeout_sec=0.01)

        finally:
            # 恢復原本終端模式
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

    def publish_joint_command(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.joint_positions
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    joint_keyboard_control = JointKeyboardControl()

    try:
        joint_keyboard_control.run()
    except KeyboardInterrupt:
        pass
    finally:
        joint_keyboard_control.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
