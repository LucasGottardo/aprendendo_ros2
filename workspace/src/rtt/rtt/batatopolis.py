from __future__ import annotations

import os
import math
import random
from pathlib import Path
from typing import List, Tuple

import numpy as np
from PIL import Image, ImageDraw
import yaml

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path as RosPath
from geometry_msgs.msg import PoseStamped

def load_map_yaml(path):
    with open(path, "r") as f:
        return yaml.safe_load(f)

def world_to_grid(x, y, origin, res):
    gx = int((x - origin[0]) / res)
    gy = int((y - origin[1]) / res)
    return gx, gy

def grid_to_world(gx, gy, origin, res):
    x = gx * res + origin[0] + res / 2.0
    y = gy * res + origin[1] + res / 2.0
    return x, y

class NodeRRT:
    def __init__(self, x: float, y: float, parent: int | None):
        self.x = x
        self.y = y
        self.parent = parent

def collision_free(p1, p2, occ: np.ndarray, origin, res):
    steps = int(math.hypot(p2[0]-p1[0], p2[1]-p1[1]) / res * 2)
    for i in range(steps + 1):
        t = i / steps
        x = p1[0] + t * (p2[0] - p1[0])
        y = p1[1] + t * (p2[1] - p1[1])
        gx, gy = world_to_grid(x, y, origin, res)
        if gx < 0 or gy < 0 or gx >= occ.shape[1] or gy >= occ.shape[0]:
            return False
        if occ[gy, gx] != 0:
            return False
    return True

def reconstruct_path(nodes: List[NodeRRT], goal_idx: int):
    path = []
    idx = goal_idx
    while idx is not None:
        n = nodes[idx]
        path.append((n.x, n.y))
        idx = n.parent
    path.reverse()
    return path

def rrt(start, goal_area, occ: np.ndarray, origin, res, step=0.3, goal_sample_rate=0.3, max_iter=10000):
    nodes = [NodeRRT(*start, None)]
    edges = []
    
    # goal_area deve ser ((x_min, y_min), (x_max, y_max))
    goal_center = (goal_area[0][0] + goal_area[1][0])/2, ((goal_area[0][1] + goal_area[1][1])/2)
    goal_radius = math.hypot(goal_area[1][0]-goal_area[0][0], goal_area[1][1]-goal_area[0][1])/2

    for _ in range(max_iter):
        # Amostragem com bias para o objetivo
        if random.random() < goal_sample_rate:
            # Amostra um ponto aleatório dentro da área do objetivo
            x_rand = random.uniform(goal_area[0][0], goal_area[1][0])
            y_rand = random.uniform(goal_area[0][1], goal_area[1][1])
        else:
            x_min, y_min = origin
            x_max = origin[0] + occ.shape[1] * res
            y_max = origin[1] + occ.shape[0] * res
            x_rand = random.uniform(x_min, x_max)
            y_rand = random.uniform(y_min, y_max)

        nearest_idx = min(range(len(nodes)), key=lambda i: (nodes[i].x-x_rand)**2 + (nodes[i].y-y_rand)**2)
        nearest_node = nodes[nearest_idx]

        theta = math.atan2(y_rand - nearest_node.y, x_rand - nearest_node.x)
        new_x = nearest_node.x + step * math.cos(theta)
        new_y = nearest_node.y + step * math.sin(theta)

        if not collision_free((nearest_node.x, nearest_node.y), (new_x, new_y), occ, origin, res):
            continue

        new_node = NodeRRT(new_x, new_y, nearest_idx)
        nodes.append(new_node)
        edges.append((nearest_node.x, nearest_node.y, new_x, new_y))

        # Verifica se o novo nó está dentro da área do objetivo
        if (goal_area[0][0] <= new_x <= goal_area[1][0] and 
            goal_area[0][1] <= new_y <= goal_area[1][1]):
            path = reconstruct_path(nodes, len(nodes)-1)
            return path, edges

    return [], edges

class RRTPlanner(Node):
    def __init__(self):
        super().__init__("rrt_planner")

        self.declare_parameter("map_yaml_path", os.path.expanduser('~/workspace/maps/map.yaml'))
        self.declare_parameter("start_x", 1.0)
        self.declare_parameter("start_y", 1.0)
        # Parâmetros da área do objetivo
        self.declare_parameter("goal_min_x", 3.8)
        self.declare_parameter("goal_min_y", 1.8)
        self.declare_parameter("goal_max_x", 4.2)
        self.declare_parameter("goal_max_y", 2.2)
        self.declare_parameter("step", 0.3)
        self.declare_parameter("max_iter", 10000)

        yaml_path = os.path.expanduser(self.get_parameter("map_yaml_path").value)
        start = (self.get_parameter("start_x").value, self.get_parameter("start_y").value)
        goal_area = [
            [self.get_parameter("goal_min_x").value, self.get_parameter("goal_min_y").value],
            [self.get_parameter("goal_max_x").value, self.get_parameter("goal_max_y").value]
        ]
        step = float(self.get_parameter("step").value)
        max_iter = int(self.get_parameter("max_iter").value)

        map_cfg = load_map_yaml(yaml_path)
        res = map_cfg["resolution"]
        origin = map_cfg["origin"][:2]
        pgm_path = os.path.join(os.path.dirname(yaml_path), map_cfg["image"])

        occ_grid = self.load_occupancy(pgm_path)
        path, edges = rrt(start, goal_area, occ_grid, origin, res, step=step, max_iter=max_iter)

        if not path:
            self.get_logger().warn("RRT não encontrou solução!")
        else:
            self.get_logger().info(f"RRT encontrou caminho com {len(path)} pontos e {len(edges)} arestas geradas")

        self.publisher_ = self.create_publisher(RosPath, "rrt_path", 10)
        path_msg = self.create_path_msg(path)
        self.publisher_.publish(path_msg)

        self.save_png(pgm_path, occ_grid, path, edges, res, origin)
        self.get_logger().info("Planner finalizado — encerrar nodo.")
        rclpy.shutdown()

    def load_occupancy(self, pgm_path: str) -> np.ndarray:
        img = Image.open(pgm_path).convert("L")
        arr = np.array(img)
        arr = np.flipud(arr)
        occ = np.zeros(arr.shape, dtype=int)
        occ[arr < 50] = 1
        occ[arr > 250] = 0
        occ[(arr >= 50) & (arr <= 250)] = -1
        return occ

    def create_path_msg(self, path: List[Tuple[float, float]]):
        msg = RosPath()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in path:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        return msg

    def save_png(self, pgm_path, occ, path, edges, res, origin):
        try:
            # Carrega a imagem base
            base_img = Image.open(pgm_path).convert("L")
            
            # Cria imagem RGB para desenho
            rgb_img = Image.merge("RGB", (base_img, base_img, base_img))
            draw = ImageDraw.Draw(rgb_img)
            
            # Obtém dimensões da imagem
            img_width, img_height = base_img.size

            # Desenha as arestas exploradas (em ciano)
            for x1, y1, x2, y2 in edges:
                p1 = world_to_grid(x1, y1, origin, res)
                p2 = world_to_grid(x2, y2, origin, res)
                # Ajuste de coordenadas Y (sistema de imagem tem Y para baixo)
                draw.line([(p1[0], img_height - p1[1]), 
                        (p2[0], img_height - p2[1])], 
                        fill=(0, 255, 255))

            # Desenha o caminho final (em vermelho)
            if path:
                prev_point = world_to_grid(*path[0], origin, res)
                for point in path[1:]:
                    curr_point = world_to_grid(*point, origin, res)
                    draw.line([(prev_point[0], img_height - prev_point[1]), 
                            (curr_point[0], img_height - curr_point[1])], 
                            fill=(255, 0, 0), width=2)
                    prev_point = curr_point

            out_path = Path("~/mapa_rrt_caminho.png").expanduser()
            rgb_img.save(out_path)
            self.get_logger().info(f"PNG salvo em {out_path}")
            
        except Exception as e:
            self.get_logger().error(f"Erro ao salvar imagem: {str(e)}")

            out_path = Path("~/mapa_rrt_caminho.png").expanduser()
            rgb_img.save(out_path)
            self.get_logger().info(f"PNG salvo em {out_path}")

        except Exception as e:
            self.get_logger().error(f"Erro ao salvar imagem: {str(e)}")
        
def main(args=None):
    rclpy.init(args=args)
    node = RRTPlanner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
