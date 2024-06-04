#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped,Quaternion
import tf2_ros
from sensor_msgs.msg import JointState
import math
from math import sin, cos, pi
import time


class Move_Arm_Node(Node):
    def __init__(self):
        super().__init__("RP_Arm")
        self.jointPublisher = self.create_publisher(JointState,"/joint_states",10)        
        self.tf_broadcaster_ = tf2_ros.TransformBroadcaster(self)

        self.jointName = ["slider_joint", "arm_joint"]
        # Define initial values 
        self.jointPosition = [0.0, 0.0]
        self.theta = 0
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

        msg_base_arm.transform.translation.z = 0.7
        msg_base_arm.transform.rotation = euler_to_quaternion(0, 0, self.theta + pi/2)

        # Publish transform for rod to slider
        msg_rod_slider = TransformStamped()
        msg_rod_slider.header.stamp.sec = seconds
        msg_rod_slider.header.stamp.nanosec = nanoseconds
        msg_rod_slider.header.frame_id = "rod_link"
        msg_rod_slider.child_frame_id = "slider_link"
        
        msg_rod_slider.transform.translation.x = self.jointPosition[0]
        msg_rod_slider.transform.rotation.w = 1.0
        
        self.tf_broadcaster_.sendTransform(msg_base_arm)
        self.key_input_callback()
        
        
    def inverse_kinematics(self):
        [self.x, self.y] = self.get_input()

        if self.y == 0.0:
            self.y = 0.00001
        self.dist = math.sqrt(pow(self.x , 2) + pow(self.y , 2))     
        self.theta = math.atan(- self.x / self.y) 
        if(self.y < 0):
            self.theta += 3.14
        return self.dist, self.theta
    
    def get_input(self):
        print("Enter end effector position")
        self.x = float(input("x:"))
        self.y = float(input("y:"))
        return self.x , self.y
    
    def key_input_callback(self):  
        [self.dist, self.theta] = self.inverse_kinematics()
        if (self.dist > -1.1 and self.dist < 1.1):
            self.jointPosition[0] = self.dist
        elif(self.dist > 1.1):
            self.jointPosition[0] = 1.0
        elif(self.dist < -1.1):
            self.jointPosition[0] = -1.0
        self.jointPosition[1] = self.theta


def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)


def main(args = None):
    rclpy.init(args = args)
    node = Move_Arm_Node()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()