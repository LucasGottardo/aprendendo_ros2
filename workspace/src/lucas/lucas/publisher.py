import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist


import rclpy
from rclpy.node import Node

distancia_objetivo = 1
p_gain = 0.1

# Cria o nó do ROS como uma clase do python
class MeuNo(Node):

    # Contrutor do nó
    def __init__(self):
        # Aqui é definido o nome do nó
        super().__init__('publisher')
        self.publisher = self.create_publisher(Twist,'cmd_vel', 10)

    #recebe as mensagens do subscriber
    def sensor_callback(self, msg):
        self.distancia_direita = msg.data
        self.calculando()
        self.talker_callback()

    # Aqui o seu nó está executando no ROS
    def run(self):

        # Executa uma iteração do loop de processamento de mensagens.
        rclpy.spin_once(self)

    #calcula a distancia de ajuste
    def calculando(self): 
        error = distancia_objetivo - self.distancia_direita
        self.power = p_gain*error

    def talker_callback(self):
        cmd = Twist()
        cmd.linear.x = 0.5 #velocidade de linha reta
        cmd.angular.z = power #velocidade angular
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
