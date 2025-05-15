#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import cv2
import os
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from queue import Queue
from ament_index_python.packages import get_package_share_directory
from wavefront.map_processor import MapProcessor

class WavefrontPlanner(Node):
    def __init__(self):
        super().__init__('wavefront_planner')
        
        # Parâmetros atualizados com o nome correto do mapa
        self.declare_parameter('map_name', 'map')  # Nome atualizado
        self.declare_parameter('inflation_radius', 0.3)
        
        self.map_processor = MapProcessor()
        
        # Carregar mapa com nome correto
        map_name = self.get_parameter('map_name').value
        map_path = os.path.join(
            get_package_share_directory('wavefront'),
            'maps',
            f'{map_name}.yaml'  # Usa o nome do parâmetro
        )
        
        try:
            self.map_data, self.map_info = self.map_processor.load_map_from_yaml(map_path)
            self.get_logger().info(f"Mapa carregado: {map_path}")
        except Exception as e:
            self.get_logger().error(f"Erro ao carregar mapa: {str(e)}")
            raise
        
        self.path_pub = self.create_publisher(Path, '/wavefront_path', 10)
        self.example_plan()

    def example_plan(self):
        """Exemplo de planejamento com coordenadas atualizadas"""
        start = Point()
        start.x = 1.0
        start.y = 1.0
        
        goal = Point()
        goal.x = 5.0
        goal.y = 5.0
        
        path = self.plan_path(start, goal)
        
        if path:
            output_dir = os.path.join(
                get_package_share_directory('wavefront'),
                'maps/output'
            )
            os.makedirs(output_dir, exist_ok=True)
            output_path = os.path.join(output_dir, 'path_result.png')
            self.save_path_image(path, output_path)

    def plan_path(self, start, goal):
        """Planejamento de caminho com verificações de segurança"""
        start_px = self.map_processor.world_to_pixel(start.x, start.y)
        goal_px = self.map_processor.world_to_pixel(goal.x, goal.y)

        if not self.valid_point(start_px) or not self.valid_point(goal_px):
            return None

        inflation_pixels = int(self.get_parameter('inflation_radius').value / 
                             self.map_info['resolution'])
        inflated_map = self.inflate_obstacles(inflation_pixels)
        
        wave_grid = self.run_wavefront(inflated_map, goal_px)
        path_pixels = self.extract_path(wave_grid, start_px, goal_px)
        
        return self.create_ros_path(path_pixels) if path_pixels else None

    def valid_point(self, point_px):
        """Validação robusta de pontos"""
        x, y = point_px
        return (
            0 <= x < self.map_data.shape[1] and
            0 <= y < self.map_data.shape[0] and
            self.map_data[y, x] < self.map_info['occupied_thresh'] * 255
        )

    def inflate_obstacles(self, inflation_pixels):
        """Inflação de obstáculos com OpenCV otimizado"""
        binary = np.where(self.map_data > self.map_info['occupied_thresh'] * 255, 1, 0).astype(np.uint8)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2*inflation_pixels+1, 2*inflation_pixels+1))
        inflated = cv2.dilate(binary, kernel)
        return np.where(inflated > 0, 100, 0)

    def run_wavefront(self, grid, goal):
        """Algoritmo Wavefront com otimizações"""
        wave_grid = np.full(grid.shape, -1)
        q = Queue()
        gx, gy = goal
        wave_grid[gy, gx] = 0
        q.put((gy, gx))

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
        """Extrai caminho com tratamento de erros"""
        path = []
        x, y = start
        current = (y, x)

        if wave_grid[current] == -1:
            self.get_logger().error("Start não alcançável!")
            return None

        directions = [(-1,-1), (-1,0), (-1,1),
                     (0,-1),          (0,1),
                     (1,-1),  (1,0),  (1,1)]

        try:
            while wave_grid[current] != 0:
                path.append((current[1], current[0]))
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
                    self.get_logger().warn("Caminho incompleto")
                    break
                current = next_cell
            
            path.append(goal)
            return path[::-1]
        except Exception as e:
            self.get_logger().error(f"Erro na extração do caminho: {str(e)}")
            return None

    def create_ros_path(self, path_pixels):
        """Cria mensagem Path do ROS2"""
        ros_path = Path()
        ros_path.header.frame_id = "map"
        ros_path.header.stamp = self.get_clock().now().to_msg()
        
        for px, py in path_pixels:
            world_x, world_y = self.map_processor.pixel_to_world(px, py)
            pose = PoseStamped()
            pose.header = ros_path.header
            pose.pose.position.x = world_x
            pose.pose.position.y = world_y
            pose.pose.orientation.w = 1.0
            ros_path.poses.append(pose)
        
        self.path_pub.publish(ros_path)
        return ros_path

    def save_path_image(self, path, output_path):
        """Salva imagem com visualização do caminho"""
        try:
            path_pixels = []
            for pose in path.poses:
                px, py = self.map_processor.world_to_pixel(
                    pose.pose.position.x,
                    pose.pose.position.y)
                path_pixels.append((px, py))

            map_img = cv2.imread(self.map_info['image'], cv2.IMREAD_GRAYSCALE)
            output_img = cv2.cvtColor(map_img, cv2.COLOR_GRAY2BGR)

            for i in range(len(path_pixels)-1):
                cv2.line(output_img, path_pixels[i], path_pixels[i+1], (0,0,255), 2)

            if path_pixels:
                cv2.circle(output_img, path_pixels[0], 5, (0,255,0), -1)
                cv2.circle(output_img, path_pixels[-1], 5, (255,0,0), -1)

            cv2.imwrite(output_path, output_img)
            self.get_logger().info(f"Rota salva em: {output_path}")
        except Exception as e:
            self.get_logger().error(f"Erro ao salvar imagem: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    planner = WavefrontPlanner()
    try:
        rclpy.spin(planner)
    except KeyboardInterrupt:
        planner.get_logger().info("Planejador encerrado")
    finally:
        planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()