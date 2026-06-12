#!/usr/bin/env python3
"""Latch a robot description (URDF string) onto a topic.

The joint_state_transformer initializes its RobotModel from a /robot_description topic
(TRANSIENT_LOCAL). It needs the *constraint* render of the robot (legs + phantom tool chain +
<constraint> loop closures), which differs from the *sim* render that robot_state_publisher
publishes for Gazebo/gz_ros2_control. This tiny node publishes a given URDF string (passed as the
'robot_description' parameter) latched on a configurable topic so the transformer can be remapped
to it.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from std_msgs.msg import String


class DescriptionPublisher(Node):
    def __init__(self):
        super().__init__('description_publisher')
        self.declare_parameter('robot_description', '')
        self.declare_parameter('topic', 'robot_description')
        urdf = self.get_parameter('robot_description').value
        topic = self.get_parameter('topic').value
        if not urdf:
            self.get_logger().error('empty robot_description parameter; nothing to publish')
        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(String, topic, qos)
        self.pub.publish(String(data=urdf))
        self.get_logger().info(f'latched robot_description ({len(urdf)} chars) on "{topic}"')


def main(args=None):
    rclpy.init(args=args)
    node = DescriptionPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()  # idempotent: avoids "rcl_shutdown already called" on SIGINT


if __name__ == '__main__':
    main()
