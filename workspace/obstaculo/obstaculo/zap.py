import rclpy
from rclpy.node import Node
import numpy
import tf_transformations
import time

from geometry_msgs.msg import Twist

from sensor_msgs.msg import LaserScan

from nav_msgs import Odometry




import rclpy
from rclpy.node import Node

x = 2.0
y = 0.0

# Cria o nó do ROS como uma clase do python
class MeuNo(Node):

    # Contrutor do nó
    def __init__(self):
        # Aqui é definido o nome do nó
        super().__init__('zap')
        self.publisher = self.create_publisher(Twist,'/cmd_vel', 10)
        self.subscription = self.create_subscription(LaserScan,'/scan', self.listener_callback, 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.Odometry_callback, 10)

    #manda as mensagens
    def listener_callback(self, msg):
        self.distancia_direita = msg.ranges[270]
        self.distancia_frente = msg.ranges[0]
        self.distancia_esquerda = msg.ranges[90]
        self.calculando_ang()
        self.talker_callback()

    # Aqui o seu nó está executando no ROS
    def run(self):
        
        # Executa uma iteração do loop de processamento de mensagens.
        rclpy.spin(self)

    #calcula a distancia de ajuste
            
    def Odometry_callback(self, msg):
        _ , _ , self.yaw = tf_transformations.euler_from_quaternion(
            [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    def talker_callback(self):
        cmd = Twist()
        cmd.linear.x = 0.2 #velocidade de linha reta
        cmd.angular.z = 0.0 #velocidade angular
        if (self.distancia_frente <= 1.0): #ajusta frontal
            cmd.linear.x = 0.0
            time.sleep(2)
            cmd.angular.z = 1.0
            time.sleep(2)
        if (self.distancia_direita <= 1.0): #ajusta direita
            cmd.linear.x = 0.0
            time.sleep(2)
            cmd.angular.z = 1.0
            time.sleep(2)
        if (self.distancia_esquerda <= 1.0): #ajusta esquerda
            cmd.linear.x = 0.0
            time.sleep(2)
            cmd.angular.z = 1.0 
            time.sleep(2)              
        self.publisher.publish(cmd)

    def caminhando(self):
        ponto_final_x = x
        ponto_final_y = y
        self.posicao_atual = 


    # Destrutor do nó
    def __del__(self):
        self.get_logger().info('Finalizando o nó! Tchau, tchau...')


# Função principal
def main(args=None):
    rclpy.init(args=args)
    node = MeuNo()
    try:
        node.run()
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
   
if __name__ == '__main__':
    main()  