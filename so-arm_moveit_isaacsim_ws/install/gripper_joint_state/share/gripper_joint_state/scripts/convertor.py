#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Joy
import threading

finger_pos = 0

class Get_joint_states(Node):
    def __init__(self):
        super().__init__('get_joint_states')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10)
        self.subscription   

        self.modified_joint_states_pub = self.create_publisher(JointState, '/isaac_joint_states', 10)

    def listener_callback(self, data):
        global finger_pos
        
        data.position[5] = finger_pos
        self.modified_joint_states_pub.publish(data)

class Joy_subscriber(Node):

    def __init__(self):
        super().__init__('joy_subscriber')
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.listener_callback,
            10)
        self.subscription

        self.C = 0.0005
        self.limit = 0.023

    def listener_callback(self, data):
        global finger_pos

        finger_pos += self.C*data.axes[5]

        if finger_pos > self.limit:
            finger_pos = self.limit
        elif finger_pos < 0:
            finger_pos = 0

if __name__ == '__main__':
    rclpy.init(args=None)

    get_joint_states = Get_joint_states()
    joy_subscriber = Joy_subscriber()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(get_joint_states)
    executor.add_node(joy_subscriber)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    rate = get_joint_states.create_rate(2)
    try:
        while rclpy.ok():
            rate.sleep()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    executor_thread.join()