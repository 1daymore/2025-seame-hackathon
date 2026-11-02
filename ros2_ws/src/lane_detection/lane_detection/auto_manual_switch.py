import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32
from piracer.vehicles import PiRacerPro

import cv2
from cv_bridge import CvBridge, CvBridgeError
from rclpy.qos import QoSProfile, ReliabilityPolicy

class MainNode(Node):
    def __init__(self):
        super().__init__('main_node')
        self.piracer = PiRacerPro()
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.is_auto = 0
        self.steer = 0
        self.throttle = 0
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.manual_steer = 0
        self.manual_throttle = 0
    
    def joy_callback(self, joy_msg):
        # 모드 변경

        # A: manual
        # B: auto
        if self.is_auto == 0 and joy_msg.buttons[1] == 1:
            self.is_auto = 1
        elif self.is_auto == 1 and joy_msg.buttons[0] == 1:
            self.is_auto = 0 
        
        # steer, throttle from joystick
        self.manual_steer = joy_msg.axes[0]
        self.manual_throttle = joy_msg.axes[4]

"""
        print(f"steer: {joy_msg.axes[0]}") # -1 ~ + 1
        print(f"throttle: {joy_msg.axes[4]}") # -1 ~ +1

        buttons_list = ['A', 'B', 'X', 'Y']
        for i in range(4):
            if joy_msg.buttons[i] == 1:
                print(buttons_list[i])
"""


    def timer_callback(self):
        if self.is_auto == 1:
            print("[Auto Mode]")
            self.steer = 0.5
            self.throttle = 0.2
        else:
            print("[Manual Mode]")
            self.steer = self.manual_steer
            self.throttle = self.manual_throttle

        self.piracer.set_steering_percent(self.steer)
        self.piracer.set_throttle_percent(self.throttle)




def main(args =None):
    rclpy.init(args=args)
    node = MainNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

