#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from queue import Queue
import os
from ament_index_python.packages import get_package_share_directory
from .map_processor import MapProcessor

from wavefront.map_processor import MapProcessor

class WavefrontPlanner(Node):
    def __init__(self):
        super().__init__('loucura')
        
        # Parâmetros
        self.declare_parameter('map_name', 'map')  # Nome base do mapa sem extensão
        self.declare_parameter('inflation_radius', 0.3)
        
        # Processador de mapa
        self.map_processor = MapProcessor()
        
        # Carregar mapa
        map_name = self.get_parameter('map_name').value
        map_path = os.path.join(
            get_package_share_directory('wavefront'),
            'maps',
            f'{map_name}.yaml')
        
        try:
            self.map_data, self.map_info = self.map_processor.load_map_from_yaml(map_path)
            self.get_logger().info(f"Mapa carregado: {map_path}")
        except Exception as e:
            self.get_logger().error(f"Erro ao carregar mapa: {str(e)}")
            raise
        
        # Publisher para o caminho
        self.path_pub = self.create_publisher(Path, '/wavefront_path', 10)
        
        # Exemplo de planejamento
        self.example_plan()
    
    def example_plan(self):
        """Executa um planejamento de exemplo"""
        # Pontos de início e objetivo
        start = Point()
        start.x = 1.0  # Coordenadas em metros
        start.y = 1.0
        
        goal = Point()
        goal.x = 5.0
        goal.y = 5.0
        
        # Planejar caminho
        path = self.plan_path(start, goal)
        
        if path:
            # Salvar imagem com o caminho
            output_dir = os.path.join(
                get_package_share_directory('wavefront'),
                'maps/output')
            os.makedirs(output_dir, exist_ok=True)
            
            output_path = os.path.join(output_dir, 'path_result.png')
            self.save_path_image(path, output_path)
    
    def plan_path(self, start, goal):
        """Planeja caminho entre start e goal (Point)"""
        # Converter para pixels
        start_px = self.map_processor.world_to_pixel(start.x, start.y)
        goal_px = self.map_processor.world_to_pixel(goal.x, goal.y)
        
        # Verificar pontos válidos
        if not self.valid_point(start_px) or not self.valid_point(goal_px):
            return None
        
        # Processar mapa
        inflation_pixels = int(self.get_parameter('inflation_radius').value / 
                          self.map_info['resolution'])
        inflated_map = self.inflate_obstacles(inflation_pixels)
        
        # Wavefront
        wave_grid = self.run_wavefront(inflated_map, goal_px)
        path_pixels = self.extract_path(wave_grid, start_px, goal_px)
        
        if not path_pixels:
            return None
        
        # Converter para Path do ROS
        return self.create_ros_path(path_pixels)
    
    def valid_point(self, point_px):
        """Verifica se ponto está no mapa e não é obstáculo"""
        x, y = point_px
        if (0 <= x < self.map_data.shape[1] and 
            0 <= y < self.map_data.shape[0] and
            self.map_data[y, x] < self.map_info['occupied_thresh'] * 255):
            return True
        self.get_logger().error("Ponto inválido!")
        return False
    
    def inflate_obstacles(self, inflation_pixels):
        """Infla obstáculos no mapa"""
        # Binarizar mapa
        binary = np.where(self.map_data > self.map_info['occupied_thresh'] * 255, 1, 0)
        
        # Criar kernel de inflação
        kernel = np.ones((2*inflation_pixels+1, 2*inflation_pixels+1), np.uint8)
        inflated = cv2.dilate(binary, kernel)
        
        return np.where(inflated > 0, 100, 0)  # 100 = obstáculo
    
    def run_wavefront(self, grid, goal):
        """Executa algoritmo wavefront"""
        wave_grid = np.full(grid.shape, -1)
        q = Queue()
        
        gx, gy = goal
        wave_grid[gy, gx] = 0
        q.put((gy, gx))
        
        # 8-vizinhança
        directions = [(-1,-1), (-1,0), (-1,1),
                     (0,-1),          (0,1),
                     (1,-1),  (1,0),  (1,1)]
        
        while not q.empty():
            y, x = q.get()
            
            for dy, dx in directions:
                ny, nx = y+dy, x+dx
                if (0 <= ny < grid.shape[0] and 
                    0 <= nx < grid.shape[1] and
                    grid[ny, nx] == 0 and 
                    wave_grid[ny, nx] == -1):
                    
                    wave_grid[ny, nx] = wave_grid[y, x] + 1
                    q.put((ny, nx))
        
        return wave_grid
    
    def extract_path(self, wave_grid, start, goal):
        """Extrai caminho do wave_grid"""
        path = []
        x, y = start
        current = (y, x)  # (row, col)
        
        if wave_grid[current] == -1:
            self.get_logger().error("Start não alcançável!")
            return None
        
        directions = [(-1,-1), (-1,0), (-1,1),
                    (0,-1),          (0,1),
                    (1,-1),  (1,0),  (1,1)]
        
        while wave_grid[current] != 0:
            path.append((current[1], current[0]))  # (x, y)
            min_val = wave_grid[current]
            next_cell = current
            
            for dy, dx in directions:
                ny, nx = current[0]+dy, current[1]+dx
                if (0 <= ny < wave_grid.shape[0] and
                    0 <= nx < wave_grid.shape[1] and
                    wave_grid[ny, nx] >= 0 and
                    wave_grid[ny, nx] < min_val):
                    
                    min_val = wave_grid[ny, nx]
                    next_cell = (ny, nx)
            
            if next_cell == current:
                break
            current = next_cell
        
        path.append(goal)
        return path[::-1]  # Inverter para começar do início
    
    def create_ros_path(self, path_pixels):
        """Cria mensagem ROS Path"""
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        
        for px, py in path_pixels:
            world_x, world_y = self.map_processor.pixel_to_world(px, py)
            
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = world_x
            pose.pose.position.y = world_y
            pose.pose.orientation.w = 1.0
            
            path.poses.append(pose)
        
        self.path_pub.publish(path)
        return path
    
    def save_path_image(self, path, output_path):
        """Salva imagem com caminho"""
        # Converter caminho para pixels
        path_pixels = []
        for pose in path.poses:
            px, py = self.map_processor.world_to_pixel(
                pose.pose.position.x,
                pose.pose.position.y)
            path_pixels.append((px, py))
        
        # Carregar mapa original
        map_img = cv2.imread(self.map_info['image'], cv2.IMREAD_GRAYSCALE)
        output_img = cv2.cvtColor(map_img, cv2.COLOR_GRAY2BGR)
        
        # Desenhar caminho
        for i in range(len(path_pixels)-1):
            cv2.line(output_img, path_pixels[i], path_pixels[i+1], (0,0,255), 2)
        
        # Pontos inicial e final
        if path_pixels:
            cv2.circle(output_img, path_pixels[0], 5, (0,255,0), -1)  # Verde
            cv2.circle(output_img, path_pixels[-1], 5, (255,0,0), -1)  # Azul
        
        cv2.imwrite(output_path, output_img)
        self.get_logger().info(f"Imagem salva em: {output_path}")

def main(args=None):
    rclpy.init(args=args)
    planner = WavefrontPlanner()
    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()