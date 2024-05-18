#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import math
import time


class Move_Arm_Node(Node):
    def __init__(self):
        super().__init__("RP_Arm")
        self.jointPublisher = self.create_publisher(JointState,"/joint_states",10)        
        self.tf_broadcaster_ = tf2_ros.TransformBroadcaster(self)

        self.jointName = ["slider_joint", "arm_joint"]
        self.jointPosition = [0.0, 0.0]
        self.z = 0.0
        self.w = 1.0
        self.get_logger().info("RP_Arm node has been started")
        
        self.timer = self.create_timer(0.1, self.go_to_target)

    def go_to_target(self):
        msg = JointState()
        timestamp = time.time()
        seconds = int(timestamp)
        nanoseconds = int((timestamp - seconds) * 1e9)
        msg.header.stamp.sec = seconds
        msg.header.stamp.nanosec = nanoseconds
        msg.name = self.jointName
        msg.position = self.jointPosition
        self.jointPublisher.publish(msg)

        # Publish transform for base to arm
        msg_base_arm = TransformStamped()
        msg_base_arm.header.stamp.sec = seconds
        msg_base_arm.header.stamp.nanosec = nanoseconds
        msg_base_arm.header.frame_id = "base_link"
        msg_base_arm.child_frame_id = "arm_link"

        msg_base_arm.transform.translation.x= 0.0
        msg_base_arm.transform.translation.y = 0.0
        msg_base_arm.transform.translation.z = 0.7

        msg_base_arm.transform.rotation.x = 0.0
        msg_base_arm.transform.rotation.y = 0.0 
        msg_base_arm.transform.rotation.z = self.z
        msg_base_arm.transform.rotation.w = self.w

        # Publish transform for rod to slider
        msg_rod_slider = TransformStamped()
        msg_rod_slider.header.stamp.sec = seconds
        msg_rod_slider.header.stamp.nanosec = nanoseconds
        msg_rod_slider.header.frame_id = "rod_link"
        msg_rod_slider.child_frame_id = "slider_link"
        
        msg_rod_slider.transform.translation.x = self.jointPosition[0]
        msg_rod_slider.transform.translation.y = 0.0
        msg_rod_slider.transform.translation.z = 0.0

        msg_rod_slider.transform.rotation.x = 0.0
        msg_rod_slider.transform.rotation.y = 0.0 
        msg_rod_slider.transform.rotation.z = 0.0
        msg_rod_slider.transform.rotation.w = 1.0

        self.tf_broadcaster_.sendTransform(msg_base_arm)

        self.key_input_callback()
        
        
    def inverse_kinematics(self):
        [self.x, self.y] = self.get_input()

        if self.y == 0.0:
            self.y = 0.00001
        self.dist = math.sqrt(pow(self.x , 2) + pow(self.y , 2))     
        self.theta = math.atan( self.x / self.y) 
        if(self.x < 0 and self.y>0):
            self.theta  = abs(self.theta) +(3.14/2)
        elif(self.x < 0 and self.y<0):
            self.theta = (3*3.14/2) - abs(self.theta)
        elif(self.x > 0 and self.y<0):
            self.theta =(3*3.14/2) + abs(self.theta)
        
        [self.z , self.w] = self.calculate_quaternion(self.theta)
        return self.dist, self.theta
    
    def get_input(self):
        print("Enter end effector position")
        self.x = float(input("x:"))
        self.y = float(input("y:"))
        return self.x , self.y

    def calculate_quaternion(self,theta_rad):
        self.w = (math.cos(math.degrees(theta_rad/2)))
        self.z = (math.sin(math.degrees(theta_rad/2)))
        if(theta_rad >= 0 and theta_rad <= 2.093):
            self.z = -self.z
        elif(theta_rad > 2.093 and theta_rad <= 3.14):
            self.w = -self.w
        return self.z , self.w
    
    def key_input_callback(self):  
        [self.dist, self.theta] = self.inverse_kinematics()
        if self.dist >= -0.4 and self.dist <= 1.0:
            self.jointPosition[0] = self.dist
        if math.radians(self.theta) >= 0 and math.radians(self.theta) <= 6.28:
            self.jointPosition[1] = self.theta


def main(args = None):
    rclpy.init(args = args)
    node = Move_Arm_Node()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()