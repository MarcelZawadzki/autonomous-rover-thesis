import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import math

class FakeOdomNode(Node):
    def __init__(self):
        super().__init__('fake_odom')
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.odom_publisher = self.create_publisher(Odometry, 'odom_cmd', 10)

        self.publish_timer = self.create_timer(0.04, self.publish_twist)
        self.last_cmd_vel_time = self.get_clock().now()
        self.last_cmd_vel_msg = Twist()

        self.cmd_timeout = 0.5

    def cmd_vel_callback(self, msg):
        self.last_cmd_vel_time = self.get_clock().now()
        self.last_cmd_vel_msg = msg

    def publish_twist(self):
        # Check if it's been more than 1 second since the last cmd_vel message
        elapsed_time = (self.get_clock().now() - self.last_cmd_vel_time).nanoseconds / 1e9
        odom_msg = Odometry()
        odom_msg.header = Header()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        if elapsed_time > self.cmd_timeout:
            # Publish a message with zero speed
            twist_zero = Twist()
            twist_zero.linear.x = 0.0
            twist_zero.angular.z = 0.0
            odom_msg.twist.twist = twist_zero
        else:
            odom_msg.twist.twist = self.last_cmd_vel_msg

        odom_msg.twist.covariance = [
            0.02**2, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.02**2, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.02**2, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.1**2, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.1**2, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1**2
        ]

        # Publish the Odometry message
        self.odom_publisher.publish(odom_msg)


def main(args=None):
    rclpy.init(args=args)
    fake_odom_node = FakeOdomNode()
    rclpy.spin(fake_odom_node)

    fake_odom_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()