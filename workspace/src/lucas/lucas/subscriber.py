import rclpy
from rclpy.node import node

import numpy

from sensors_msgs.msg import LaserScan

distancia_direita = numpy.array(self.laser[80:100]).mean()

class MeuNo(Node):

    # Contrutor do nó
    def __init__(self):
        # Aqui é definido o nome do nó
        super().__init__('listener')
        self.subscription = self.create_subscription(String, distancia_direita , self.listener_callback,qos_profile)

    # Aqui o seu nó está executando no ROS
    def run(self):

        # Executa uma iteração do loop de processamento de mensagens.
        rclpy.spin_once(self)

    def listener_callback(self, msg):
        self.get_logger().info('mensagem publicada: "%s"' %msg.data)

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


