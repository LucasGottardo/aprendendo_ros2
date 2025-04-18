import rclpy
from rclpy.node import Node
import numpy as np
import tf_transformations
import time 


from geometry_msgs.msg import Twist

from sensor_msgs.msg import LaserScan

from nav_msgs.msg import Odometry

import rclpy
from rclpy.node import Node


# Cria o nó do ROS como uma clase do python
class MeuNo(Node):

    # Contrutor do nó
    def __init__(self):
        super().__init__('zap')
        self.publisher = self.create_publisher(Twist,'/cmd_vel', 10)
        self.subscription = self.create_subscription(LaserScan,'/scan', self.listener_callback, 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.Odometry_callback, 10)
        self.objetivo = np.array([5.0,3.0])

        self.desviando = False
        self.tempo_inicio_desvio = None
        self.pode_desviar_novamente = True
        self.tempo_cooldown = 3.0

    #manda as mensagens
    def listener_callback(self, msg):
        self.distancia_direita = msg.ranges[270]
        self.distancia_frente = msg.ranges[0]
        self.distancia_esquerda = msg.ranges[90]
        self.talker_callback()

    # Aqui o seu nó está executando no ROS
    def run(self):
        
        # Executa uma iteração do loop de processamento de mensagens.
        rclpy.spin(self)

    #informa a posição atual        
    def Odometry_callback(self, msg):
        _ , _ , self.yaw = tf_transformations.euler_from_quaternion(
            [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    def talker_callback(self):
        cmd = Twist()
        destino = self.objetivo - np.array([self.x, self.y])
        distancia_final = np.linalg.norm(destino)
        theta = np.arctan2(destino[1], destino[0])
        ajustar_angulo = np.arctan2(np.sin(theta - self.yaw), np.cos(theta - self.yaw))

        if (distancia_final <= 0.2):
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.publisher.publish(cmd)
            print(distancia_final)

        if not self.desviando and (self.distancia_frente <= 1.0 or self.distancia_direita <= 0.5 or self.distancia_esquerda <= 0.5):
            self.desviando = True
            self.tempo_inicio_desvio = time.time()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.posicao_temp_x = [(self.x + 1.5, self.y)]
            self.posicao_temp_y = [(self.x, self.y + 1.5)]
            self.publisher.publish(cmd)
            return
    
        if self.desviando:
            if time.time() - self.tempo_inicio_desvio < 2:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.2
                self.publisher.publish(cmd)
                return
        else:
            self.desviando = False
            self.tempo_fim_desvio = time.time()
            self.pode_desviar_novamente = False
            return
        if not self.pode_desviar_novamente:
            if time.time() - self.tempo_fim_desvio > self.tempo_cooldown:
                self.pode_desviar_novamente = True
        else:
            self.publisher.publish(cmd)
            return




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