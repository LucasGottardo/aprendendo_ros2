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

        theta = np.arctan2(destino[1], destino[0]) #ajusta a angulação da direção que o robo deve seguir
        ajustar_angulo = theta - self.yaw
        ajustar_angulo = np.arctan2(np.sin(ajustar_angulo), np.cos(ajustar_angulo))

        if abs(ajustar_angulo > 0.1):
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5

        if (distancia_final <= 0.2):
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.publisher.publish(cmd)
            print(distancia_final)
<<<<<<< HEAD
        
        if (self.distancia_frente <= 0.5):
            cmd.linear.x = 0.0
            cmd.linear.z = 0.5

        if (self.distancia_direita <= 0.5): #ajusta direita
=======

        if not self.desviando and (self.distancia_frente <= 1.0 or self.distancia_direita <= 0.5 or self.distancia_esquerda <= 0.5):
            self.desviando = True
            self.tempo_inicio_desvio = time.time()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.2
            self.publisher.publish(cmd)
            return
    
        if self.desviando:
            if time.time() - self.tempo_inicio_desvio < 2:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.5
                self.publisher.publish(cmd)
                return
            else:
                self.desviando = False
        
        if abs(ajustar_angulo) > 0.1:
>>>>>>> 2e2e864c7e188d78560b6a219f3edce1803a16a5
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5
        else:
            cmd.linear.x = 0.2
            cmd.angular.z = 0.0
            
        self.publisher.publish(cmd)


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
<<<<<<< HEAD
    main()
=======
    main()  
>>>>>>> 2e2e864c7e188d78560b6a219f3edce1803a16a5
