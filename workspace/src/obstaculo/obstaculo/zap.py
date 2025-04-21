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
        self.obstacle_distance = 0.8  # Distância mínima para detecção frontal (metros)
        self.rotation_time = 1.5      # Tempo de rotação inicial (segundos)
        self.linear_speed = 0.2       # Velocidade linear normal (m/s)
        self.angular_speed = 0.3      # Velocidade angular de rotação (rad/s)
        self.cooldown = 2.0           # Tempo entre desvios (segundos)

        # Variáveis de estado
        self.state = MOVING
        self.last_rotation_time = 0.0
        self.robot_position = np.array([0.0, 0.0])
        self.robot_yaw = 0.0

    def odom_callback(self, msg):
        # Atualiza posição e orientação
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
        # Detecta obstáculos na frente
        self.front_distance = min(msg.ranges[0:30] + msg.ranges[-30:])
        self.drive_controller()

    def calculate_target_angle(self):
        # Vetor para o objetivo
        direction = self.target - self.robot_position
        return np.arctan2(direction[1], direction[0])

    def angle_difference(self, target):
        # Calcula diferença angular normalizada [-pi, pi]
        diff = target - self.robot_yaw
        return np.arctan2(np.sin(diff), np.cos(diff))

    def drive_controller(self):
        cmd = Twist()
        current_time = time.time()

        # Verifica se chegou ao objetivo
        if np.linalg.norm(self.target - self.robot_position) < 0.2:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)
            self.get_logger().info("Objetivo alcançado!")
            return

        # Máquina de estados principal
        if self.state == MOVING:
            if self.front_distance < self.obstacle_distance:
                self.get_logger().info("Obstáculo detectado! Iniciando desvio...")
                self.state = ROTATING
                self.last_rotation_time = current_time
            else:
                # Controle proporcional para o objetivo
                target_angle = self.calculate_target_angle()
                angle_error = self.angle_difference(target_angle)
                
                cmd.linear.x = self.linear_speed
                cmd.angular.z = 0.5 * angle_error  # Ganho proporcional

        elif self.state == ROTATING:
            # Rotaciona por tempo fixo + verificação do sensor
            if (current_time - self.last_rotation_time > self.rotation_time) and \
               (self.front_distance > self.obstacle_distance * 1.5):
                self.get_logger().info("Realinhando com o objetivo...")
                self.state = REALIGN
                self.last_rotation_time = current_time
            else:
                cmd.angular.z = self.angular_speed  # Gira no sentido horário

        elif self.state == REALIGN:
            # Ajusta o ângulo para o objetivo
            target_angle = self.calculate_target_angle()
            angle_error = self.angle_difference(target_angle)
            
            if abs(angle_error) < 0.1:  # 0.1 rad ~ 5.7 graus
                self.get_logger().info("Realinhamento concluído!")
                self.state = MOVING
                self.last_rotation_time = current_time  # Inicia cooldown
            else:
                cmd.angular.z = 0.75 * angle_error  # Ganho mais agressivo

        # Aplica comandos
        self.cmd_vel_pub.publish(cmd)

        # Cooldown para evitar oscilações
        if (current_time - self.last_rotation_time > self.cooldown) and \
           (self.state != ROTATING):
            self.state = MOVING

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