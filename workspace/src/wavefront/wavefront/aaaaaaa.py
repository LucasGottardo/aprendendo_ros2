#Ola professor, eu fiz este codigo para ao invez de rodar no gazebo, apenas gerar uma imagem pgn exibindo o caminho escolhido
#para rodar o codigo estou usando o comando ros2 run wavefront aaaaaaa --ros-args   -p map_yaml_path:=/home/robot/workspace/install/wavefront/share/wavefront/maps/map.yaml
#porem o mapa é gerado na maquina virtual e para visualiza-lo é necessario transferilo para a memoria do computador em si
#para isso usei o comando do terminal fora da maquina virtual docker cp ros-env:/home/robot/mapa_com_caminho.png .
#isso deve abrir o mapa :), eu espero
#como ultima coisa coloquei o mapa que eu gerei ali (juro que eu nao pintei no paint)

import numpy as np
import yaml
from PIL import Image
from collections import deque
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import os
import subprocess

def mundo_para_indice(x, y, origem, resolucao):
    iy = int((y - origem[1]) / resolucao)  # linha
    ix = int((x - origem[0]) / resolucao)  # coluna
    return iy, ix

class fofinho(Node):
    def __init__(self):
        super().__init__('planejador_wavefront')
        self.publisher_ = self.create_publisher(Path, 'wavefront_path', 10)

        # Carrega parâmetros
        self.declare_parameter('map_yaml_path', os.path.expanduser('~/workspace/maps/map.yaml'))
        caminho_yaml = self.get_parameter('map_yaml_path').get_parameter_value().string_value
        self.get_logger().info(f'Usando arquivo YAML: {caminho_yaml}')

        # Carrega YAML e imagem do mapa
        mapa_yaml = self.ler_yaml(caminho_yaml)
        caminho_pgm = os.path.join(os.path.dirname(caminho_yaml), mapa_yaml['image'])
        resolucao = mapa_yaml['resolution']
        origem = mapa_yaml['origin']

        # Posições do robô e destino (em metros)
        inicio_real = (1.0, 1.0)
        meta_real = (7.0, 4.0)

        # Carrega imagem e gera mapa de ocupação
        mapa = self.carregar_pgm(caminho_pgm)
        ocupacao = self.criar_ocupados(mapa)

        self.get_logger().info(f"Shape mapa: {mapa.shape}")
        self.get_logger().info(f"Origem: {origem}")
        self.get_logger().info(f"Resolução: {resolucao}")

        # Converte posições reais para índices de matriz
        inicio = mundo_para_indice(*inicio_real, origem, resolucao)
        meta = mundo_para_indice(*meta_real, origem, resolucao)

        self.get_logger().info(f"Início real: {inicio_real} -> índice: {inicio}")
        self.get_logger().info(f"Meta real:  {meta_real} -> índice: {meta}")

        # Aplica algoritmo Wavefront
        grid_wave = self.wavefront(ocupacao, meta)
        caminho = self.extrair_caminho(grid_wave, inicio)

        self.get_logger().info(f'Caminho contém {len(caminho)} pontos')

        # Publica o caminho no tópico
        path_msg = self.criar_path_msg(caminho, resolucao, origem)
        self.publisher_.publish(path_msg)
        self.get_logger().info('Caminho publicado!')

        # Salva a imagem com o caminho traçado
        self.salvar_imagem_com_caminho(mapa, caminho)

    def ler_yaml(self, caminho_arquivo):
        with open(caminho_arquivo, 'r') as f:
            return yaml.safe_load(f)

    def carregar_pgm(self, caminho_pgm):
        img = Image.open(caminho_pgm).convert('L')
        mapa = np.array(img)
        mapa = np.flipud(mapa)  # Corrigir orientação
        return mapa

    def criar_ocupados(self, mapa, limiar=50):
        ocupacao = np.full(mapa.shape, 0)
        ocupacao[mapa < limiar] = 1
        ocupacao[mapa > 250] = 0
        ocupacao[(mapa >= limiar) & (mapa <= 250)] = -1
        return ocupacao

    def wavefront(self, ocupacao, goal):
        grid = np.copy(ocupacao)
        queue = deque()
        gx, gy = goal
        grid[gx, gy] = 2
        queue.append((gx, gy))
        while queue:
            x, y = queue.popleft()
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nx, ny = x + dx, y + dy
                if 0 <= nx < grid.shape[0] and 0 <= ny < grid.shape[1]:
                    if grid[nx, ny] == 0:
                        grid[nx, ny] = grid[x, y] + 1
                        queue.append((nx, ny))
        return grid

    def extrair_caminho(self, grid, start):
        path = []
        x, y = start
        if grid[x, y] <= 1:
            self.get_logger().warn("Posição inicial inválida.")
            return []
        while grid[x, y] != 2:
            path.append((x, y))
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nx, ny = x + dx, y + dy
                if 0 <= nx < grid.shape[0] and 0 <= ny < grid.shape[1]:
                    if grid[nx, ny] < grid[x, y] and grid[nx, ny] > 1:
                        x, y = nx, ny
                        break
        path.append((x, y))
        return path

    def criar_path_msg(self, caminho, resolucao, origem):
        path = Path()
        path.header.frame_id = 'map'
        for px, py in caminho:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = py * resolucao + origem[0] + resolucao / 2
            pose.pose.position.y = px * resolucao + origem[1] + resolucao / 2
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        return path

    def salvar_imagem_com_caminho(self, mapa_original, caminho, nome_arquivo='~/mapa_com_caminho.png'):
        imagem_colorida = np.stack((mapa_original,)*3, axis=-1)

        for px, py in caminho:
            if 0 <= px < imagem_colorida.shape[0] and 0 <= py < imagem_colorida.shape[1]:
                imagem_colorida[px, py] = [255, 0, 0]

        imagem_colorida = np.flipud(imagem_colorida)
        img = Image.fromarray(imagem_colorida.astype(np.uint8))

        caminho_saida = os.path.expanduser(nome_arquivo)
        img.save(caminho_saida)

        self.get_logger().info(f'Imagem com caminho salva em: {caminho_saida}')
        self.get_logger().info(f'Mapa shape: {mapa_original.shape}, tipo: {mapa_original.dtype}, max: {np.max(mapa_original)}, min: {np.min(mapa_original)}')

        try:
            subprocess.run(['xdg-open', caminho_saida], check=False)
        except Exception as e:
            self.get_logger().warn(f'Não foi possível abrir a imagem automaticamente: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = fofinho()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
