import rclpy
from rclpy.node import node

import numpy

from sensor_msgs.msg import LaserScan


class MeuNo(Node):

    # Contrutor do nó
    def __init__(self):
        # Aqui é definido o nome do nó
        super().__init__('subscriber')
        self.subscription = self.create_subscription(LaserScan,'scan', self.listener_callback, 10)


    # Aqui o seu nó está executando no ROS
    def run(self):

        # Executa uma iteração do loop de processamento de mensagens.
        rclpy.spin_once(self)

    def listener_callback(self, msg):
        distancia_direita = numpy.array(self.laser[80:100]).mean()

        self.get_logger().info('Distancia: {distancia_direita:.2f}' )

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


