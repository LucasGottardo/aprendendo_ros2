import rclpy
from rclpy.node import Node
import numpy

from geometry_msgs.msg import Twist

from sensor_msgs.msg import LaserScan




import rclpy
from rclpy.node import Node

distancia_objetivo = 0.4
p_gain = 0.6

# Cria o nó do ROS como uma clase do python
class MeuNo(Node):

    # Contrutor do nó
    def __init__(self):
        # Aqui é definido o nome do nó
        super().__init__('publisher')
        self.publisher = self.create_publisher(Twist,'cmd_vel', 10)
        self.subscription = self.create_subscription(LaserScan,'scan', self.listener_callback, 10)

    #manda as mensagens
    def listener_callback(self, msg):
        self.distancia_direita = msg.ranges[270]
        self.distancia_frente = msg.ranges[0]
        self.calculando()
        self.talker_callback()

    # Aqui o seu nó está executando no ROS
    def run(self):
        
        # Executa uma iteração do loop de processamento de mensagens.
        rclpy.spin(self)

    #calcula a distancia de ajuste
    def calculando(self): 
        error = distancia_objetivo - self.distancia_direita
        self.power = p_gain*error
        if (numpy.isnan(self.power)):
            self.power = 0.0
        if (numpy.isinf(self.power)):
            self.power = 0.0

    def talker_callback(self):
        cmd = Twist()
        cmd.linear.x = 0.2 #velocidade de linha reta
        cmd.angular.z = self.power #velocidade angular
        if (self.distancia_frente <= 0.4):
                cmd.linear.x = 0.0
                cmd.angular.z = 1.0
               
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
    main()   
