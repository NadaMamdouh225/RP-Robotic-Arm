#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState
from geometry_msgs.msg import Twist
import time


class my_Node(Node):
    def __init__(self):
        super().__init__("RP_Arm")
        self.jointPublisher = self.create_publisher(JointState,"/joint_states",10)
        self.keyboardSubscriber = self.create_subscription(Twist,"/cmd_vel", self.key_input_callback, 10)
        self.tf_broadcaster_ = tf2_ros.TransformBroadcaster(self)

        self.jointName = ["slider_joint", "arm_joint"]
        self.jointPosition = [0.0, 0.0]

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
        msg_base_arm.transform.rotation.z = 0.0
        msg_base_arm.transform.rotation.w = 1.0

        self.tf_broadcaster_.sendTransform(msg_base_arm)

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

        self.tf_broadcaster_.sendTransform(msg_rod_slider)

    def key_input_callback(self, msg):         
        key_slider = msg.linear.x
     #   if key == 'w':
     #       self.jointPosition[1] += 0.05  
     #   elif key == 's':
     #       self.jointPosition[1] -= 0.05
        if key_slider < 0:
            if(self.jointPosition[0] < 0.6):
                self.jointPosition[0] += 0.05 
            else:
                self.jointPosition[0] += 0.0
        elif key_slider > 0:
            if(self.jointPosition[0] > -0.6):
                self.jointPosition[0] -= 0.05 
            else:
                self.jointPosition[0] += 0.0

    def get_input(self):
        self.msg = str(input())
        return self.msg[0]

def main(args = None):
    rclpy.init(args = args)
    node = my_Node()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()