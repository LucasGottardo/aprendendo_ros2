import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path


class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')

        self.declare_parameter("linear_speed", 0.2)
        self.declare_parameter("angular_speed", 1.0)
        self.declare_parameter("goal_tolerance", 0.1)

        self.linear_speed = self.get_parameter("linear_speed").value
        self.angular_speed = self.get_parameter("angular_speed").value
        self.goal_tolerance = self.get_parameter("goal_tolerance").value

        self.path = []
        self.current_goal_idx = 0

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_sub = self.create_subscription(Path, 'rrt_path', self.path_callback, 10)
        self.timer = self.create_timer(0.1, self.control_loop)

    def path_callback(self, msg: Path):
        self.get_logger().info(f"Recebido caminho com {len(msg.poses)} pontos.")
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.current_goal_idx = 0

    def control_loop(self):
        if self.current_goal_idx >= len(self.path):
            self.cmd_pub.publish(Twist())  
            return

        robot_x, robot_y, robot_yaw = 0.0, 0.0, 0.0

        goal_x, goal_y = self.path[self.current_goal_idx]
        dx = goal_x - robot_x
        dy = goal_y - robot_y
        dist = math.hypot(dx, dy)
        angle_to_goal = math.atan2(dy, dx)
        angle_diff = angle_to_goal - robot_yaw

        
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi

        cmd = Twist()
        if abs(angle_diff) > 0.1:
            cmd.angular.z = self.angular_speed if angle_diff > 0 else -self.angular_speed
        elif dist > self.goal_tolerance:
            cmd.linear.x = self.linear_speed
        else:
            self.get_logger().info(f"Ponto {self.current_goal_idx+1} alcançado.")
            self.current_goal_idx += 1

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
