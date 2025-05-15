import rclpy
from rclpy.node import Node
import numpy as np
import tf_transformations
import time

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

# Estados do robô
MOVING = "MOVING"
ROTATING = "ROTATING"
AVOID_FORWARD = "AVOID_FORWARD"
REALIGN = "REALIGN"

class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')
        
        # Publishers e Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Parâmetros do sistema
        self.target = np.array([5.0, 3.0])  # Ponto objetivo (x, y)
        self.obstacle_distance = 0.8       # Distância mínima para detecção frontal (metros)
        self.linear_speed = 0.2            # Velocidade linear (m/s)
        self.angular_speed = 0.3           # Velocidade angular (rad/s)

        # Variáveis de estado
        self.state = MOVING
        self.robot_position = np.array([0.0, 0.0])
        self.robot_yaw = 0.0
        self.front_distance = float('inf')

        # Controle de desvio
        self.avoid_start_yaw = 0.0
        self.avoid_target_yaw = 0.0
        self.avoid_forward_start_time = 0.0

    def odom_callback(self, msg):
        self.robot_position = np.array([msg.pose.pose.position.x, 
                                        msg.pose.pose.position.y])
        quaternion = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]
        _, _, self.robot_yaw = tf_transformations.euler_from_quaternion(quaternion)

    def scan_callback(self, msg):
        self.front_distance = min(msg.ranges[0:30] + msg.ranges[-30:])
        self.drive_controller()

    def calculate_target_angle(self):
        direction = self.target - self.robot_position
        return np.arctan2(direction[1], direction[0])

    def angle_difference(self, target):
        diff = target - self.robot_yaw
        return np.arctan2(np.sin(diff), np.cos(diff))

    def drive_controller(self):
        cmd = Twist()
        current_time = time.time()

        # Objetivo alcançado
        if np.linalg.norm(self.target - self.robot_position) < 0.2:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)
            self.get_logger().info("Objetivo alcançado!")
            return

        # Máquina de estados principal
        if self.state == MOVING:
            if self.front_distance < self.obstacle_distance:
                self.get_logger().info("Obstáculo detectado! Iniciando rotação de desvio...")
                self.state = ROTATING
                self.avoid_target_yaw = 0.0  # Reset
            else:
                target_angle = self.calculate_target_angle()
                angle_error = self.angle_difference(target_angle)
                cmd.linear.x = self.linear_speed
                cmd.angular.z = 0.5 * angle_error

        elif self.state == ROTATING:
            # Inicializa a rotação para 90 graus (horário)
            if self.avoid_target_yaw == 0.0:
                self.avoid_start_yaw = self.robot_yaw
                self.avoid_target_yaw = self.avoid_start_yaw - np.pi / 2
                self.avoid_target_yaw = np.arctan2(np.sin(self.avoid_target_yaw), np.cos(self.avoid_target_yaw))

            angle_error = self.angle_difference(self.avoid_target_yaw)

            if abs(angle_error) < 0.05:
                self.get_logger().info("Rotação de 90 concluída. Avançando por 3 segundos...")
                self.state = AVOID_FORWARD
                self.avoid_forward_start_time = current_time
                self.avoid_target_yaw = 0.0  # Reset
            else:
                cmd.angular.z = -self.angular_speed  # Sentido horário

        elif self.state == AVOID_FORWARD:
            if current_time - self.avoid_forward_start_time < 3.0:
                cmd.linear.x = self.linear_speed
            else:
                self.get_logger().info("Desvio concluído. Realinhando com o objetivo...")
                self.state = REALIGN

        elif self.state == REALIGN:
            target_angle = self.calculate_target_angle()
            angle_error = self.angle_difference(target_angle)

            if abs(angle_error) < 0.1:
                self.get_logger().info("Realinhamento concluído. Retomando rota.")
                self.state = MOVING
            else:
                cmd.angular.z = 0.75 * angle_error

        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    controller = ObstacleAvoidance()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass

    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()