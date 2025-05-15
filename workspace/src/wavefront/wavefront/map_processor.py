import yaml
import cv2
import numpy as np
from pathlib import Path

class MapProcessor:
    def __init__(self):
        self.map_data = None
        self.map_info = None
    
    def load_map_from_yaml(self, yaml_path):
        """Carrega mapa do arquivo YAML"""
        with open(yaml_path, 'r') as f:
            map_yaml = yaml.safe_load(f)
        
        # Carregar imagem
        map_dir = Path(yaml_path).parent
        image_path = map_dir / map_yaml['image']
        self.map_data = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
        
        if self.map_data is None:
            raise FileNotFoundError(f"Imagem do mapa não encontrada: {image_path}")
        
        # Processar conforme YAML
        if map_yaml.get('negate', 0):
            self.map_data = 255 - self.map_data
        
        # Armazenar metadados
        self.map_info = {
            'resolution': map_yaml['resolution'],
            'origin': map_yaml['origin'],
            'occupied_thresh': map_yaml['occupied_thresh'],
            'free_thresh': map_yaml['free_thresh'],
            'image': str(image_path)
        }
        
        return self.map_data, self.map_info
    
    def world_to_pixel(self, world_x, world_y):
        """Converte coordenadas do mundo para pixels"""
        origin = self.map_info['origin']
        res = self.map_info['resolution']
        
        px = int((world_x - origin[0]) / res)
        py = int((world_y - origin[1]) / res)
        
        return (px, py)
    
    def pixel_to_world(self, px, py):
        """Converte pixels para coordenadas do mundo"""
        origin = self.map_info['origin']
        res = self.map_info['resolution']
        
        x = px * res + origin[0]
        y = py * res + origin[1]
        
        return (x, y)