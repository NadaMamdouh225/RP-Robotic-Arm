#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist

import serial

class Joints(Node):
    def __init__(self):
        super().__init__("node_pi")
        
        #self.motors_subscriber = self.create_subscription(Twist, "/cmd_vel", self.callback_motors_subscriber, 10)
        self.joints_subscriber = self.create_subscription(JointState, "/joint_states", self.callback_joints_subscriber, 10)
        
        self.get_logger().info("Serial Node Has Started")
    
    def callback_joints_subscriber(self, msg):
        print(" ")
        self.get_logger().info("Joints Message is Subscribed")
        print(msg)
        
        message1 = str(int(msg.position[0] * 100))
        message2 = str(int(msg.position[1] * 360 / 6.28))
        print("")
        print(message1)
        print(message2)
        print("")
        print("Sending Joints Commands...")
        #ser.write(message1.encode('ascii'))
        #ser.write(message2.encode('ascii'))
        print("Sending Finished...\n")

def main(args=None):
    com = "/dev/ttyACM0"
    #global ser
    #ser = serial.Serial(port=com, baudrate=115200)
    
    rclpy.init(args=args)
    node = Joints()
    rclpy.spin(node)
    rclpy.shutdown()
    
    pass

if __name__ == "__main__":
    main()
    