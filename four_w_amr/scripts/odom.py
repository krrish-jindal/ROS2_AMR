#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32MultiArray
from math import cos, sin
import tf2_ros


class OdomCalculator(Node):

    def __init__(self):
        super().__init__('odom_calculator')
        
        self.ticks_meter = 887.5  # You need to set this value based on your encoder ticks and wheel diameter
        self.base_width = 0.39  # Set the base width of your 4WD robot

        self.enc_left = 0.0
        self.enc_right = 0.0
        self.drive_constant = 1.0
        self.left = 0.0
        self.right = 0.0

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.dx = 0.0
        self.dr = 0.0

        self.base_frame_id = 'base_link'
        self.odom_frame_id = 'odom'

        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.encoder_subscription = self.create_subscription(
            Int32MultiArray,
            'encoderdata',
            self.encoder_callback,
            10)

        self.timer = self.create_timer(0.1, self.update)

    def encoder_callback(self, msg):

        # Assuming your encoder data is received as a list [left, right]
        left1 = msg.data[0]
        left2 = msg.data[1]
        right1 = msg.data[2]
        right2 = msg.data[3]

        # self.left =(left1+left2)/2 * self.drive_constant
        # self.right = (right1+right2)/2 * self.drive_constant

        self.left =left1 * self.drive_constant
        self.right = right1 * self.drive_constant
    def update(self):
        print("--------------")
        print(self.left,self.right)

        now = self.get_clock().now()

        elapsed = (now - self.get_clock().now()).nanoseconds / 1e9

        # calculate odometry
        if self.enc_left and self.enc_right is None:
            d_left = 0
            d_right = 0
        else:
            d_left = (self.left - self.enc_left) / self.ticks_meter
            d_right = (self.right - self.enc_right) / self.ticks_meter
        self.enc_left = self.left
        self.enc_right = self.right

        # distance traveled is the average of the two wheels
        d = (d_left + d_right) / 2
        # this approximation works (in radians) for small angles
        th = (d_right - d_left) / self.base_width
        # calculate velocities
        self.dx = d / elapsed
        self.dr = th / elapsed

        if d != 0:
            # calculate distance traveled in x and y
            x = cos(th) * d
            y = -sin(th) * d
            # calculate the final position of the robot
            self.x = self.x + (cos(self.th) * x - sin(self.th) * y)
            self.y = self.y + (sin(self.th) * x + cos(self.th) * y)
        if th != 0:
            self.th = self.th + th

        # publish the odom information
        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = sin(self.th / 2)
        quaternion.w = cos(self.th / 2)

        # Publish Transform
        transform = TransformStamped()
        transform.header.stamp = now.to_msg()
        transform.header.frame_id = self.odom_frame_id
        transform.child_frame_id = self.base_frame_id
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation = quaternion
        self.tf_broadcaster.sendTransform(transform)
        

        # Publish Odometry
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quaternion
        odom.child_frame_id = self.base_frame_id
        odom.twist.twist.linear.x = self.dx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.dr
        self.odom_publisher.publish(odom)


def main(args=None):
    rclpy.init(args=args)

    odom_calculator = OdomCalculator()

    rclpy.spin(odom_calculator)

    odom_calculator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
