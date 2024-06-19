#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class RECONBOTControlNode(Node):

    def __init__(self):
        super().__init__('reconbot_control')

        self.left_horizontal_propeller_pub = self.create_publisher(Float64, '/left_horizontal_propeller_cmd', 10)
        self.right_horizontal_propeller_pub = self.create_publisher(Float64, '/right_horizontal_propeller_cmd', 10)

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

    def cmd_vel_callback(self, msg):
        # Compute propeller commands from Twist message
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z

        left_horizontal_propeller_cmd = Float64()
        right_horizontal_propeller_cmd = Float64()

        left_horizontal_propeller_cmd.data = linear_velocity - angular_velocity
        right_horizontal_propeller_cmd.data = linear_velocity + angular_velocity

        self.left_horizontal_propeller_pub.publish(left_horizontal_propeller_cmd)
        self.right_horizontal_propeller_pub.publish(right_horizontal_propeller_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = RECONBOTControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
