import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path


class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')
        self.path_sub = self.create_subscription(Path, 'rrt_path', self.path_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd.vel', 10)

        self.linear.x = 0.2
        self.angular.z = 0.0
        self.goal_tolerance = 0.1

    def andandinho (self, msg: Path):
        self.get_logger().info(f"caminho recebido com {len[msg.poses]} pontos")
        


def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
