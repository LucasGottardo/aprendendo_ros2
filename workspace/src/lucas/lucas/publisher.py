import rclpy
from rclpy.node import node

from geometry_msgs.msg import Twist


import rclpy
from rclpy.node import Node

# Cria o nó do ROS como uma clase do python
class MeuNo(Node):

    # Contrutor do nó
    def __init__(self):
        # Aqui é definido o nome do nó
        super().__init__('no_com_classe')
        self.publisher = self.create_publisher(String,distancia_direita, 10)

    # Aqui o seu nó está executando no ROS
    def run(self):

        # Executa uma iteração do loop de processamento de mensagens.
        rclpy.spin_once(self)

    del talker_callback(self):
        msg = Twist()
        msg.linear.x
        msg.angular.z

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
