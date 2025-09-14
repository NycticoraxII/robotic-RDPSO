#!/usr/bin/env python

import numpy as np
from PIL import Image
import yaml
import os

def create_map_from_world():
    world_width = 20.0
    world_height = 20.0
    resolution = 0.05
    
    pixel_width = int(world_width / resolution)
    pixel_height = int(world_height / resolution)
    
    print("MAP: {}x{} ".format(pixel_width, pixel_height))
    print("RATIO: {} m/pixel".format(resolution))
    
    map_array = np.full((pixel_height, pixel_width), 255, dtype=np.uint8)
    
    def world_to_pixel(x, y):
        pixel_x = int((x + world_width/2) / resolution)
        pixel_y = int((world_height/2 - y) / resolution)
        return pixel_x, pixel_y
    
    def draw_rectangle(center_x, center_y, size_x, size_y, value=0):
        half_size_x = size_x / 2
        half_size_y = size_y / 2
        
        x1 = center_x - half_size_x
        x2 = center_x + half_size_x
        y1 = center_y - half_size_y
        y2 = center_y + half_size_y
        
        px1, py1 = world_to_pixel(x1, y2)
        px2, py2 = world_to_pixel(x2, y1)
        
        px1 = max(0, min(pixel_width-1, px1))
        px2 = max(0, min(pixel_width-1, px2))
        py1 = max(0, min(pixel_height-1, py1))
        py2 = max(0, min(pixel_height-1, py2))
        
        map_array[py1:py2+1, px1:px2+1] = value
        print("OBS: CENTRE({:.1f}, {:.1f}), SIZE({:.1f}x{:.1f})".format(
            center_x, center_y, size_x, size_y))
    
    print("\nWALL...")
    draw_rectangle(0, 10, 20, 0.2)
    draw_rectangle(0, -10, 20, 0.2)
    draw_rectangle(10, 0, 0.2, 20)
    draw_rectangle(-10, 0, 0.2, 20)
    
    print("\nOBS...")
    obstacles = [
        (2, 3, 0.8, 0.8),      # obstacle_1
        (-3, -2, 0.6, 0.6),    # obstacle_2
        (5, -4, 0.7, 0.7),     # obstacle_3
        (-6, 4, 0.5, 0.5),     # obstacle_4
        (7, 2, 0.9, 0.9),      # obstacle_5
        (-1, -6, 0.6, 0.6),    # obstacle_6
        (4, 6, 0.8, 0.8),      # obstacle_7
        (-7, -1, 0.55, 0.55),  # obstacle_8
        (1, -8, 0.7, 0.7),     # obstacle_9
        (-4, 7, 0.65, 0.65),   # obstacle_10
        (8, -6, 0.4, 0.4),     # obstacle_11
        (-8, -7, 0.5, 0.5),    # obstacle_12
    ]
    
    for i, (x, y, size_x, size_y) in enumerate(obstacles, 1):
        draw_rectangle(x, y, size_x, size_y)
    
    target_x, target_y = 3.4, -4.0
    px, py = world_to_pixel(target_x, target_y)
    if 0 <= px < pixel_width and 0 <= py < pixel_height:
        radius_pixels = int(0.3 / resolution)  # 0.3m radius
        y_indices, x_indices = np.ogrid[:pixel_height, :pixel_width]
        mask = (x_indices - px)**2 + (y_indices - py)**2 <= radius_pixels**2
        print(" ({:.1f}, {:.1f})".format(target_x, target_y))
    
    return map_array, resolution, world_width, world_height

def save_map(map_array, resolution, world_width, world_height, map_name="obstacle_world"):
    pgm_filename = "{}.pgm".format(map_name)
    image = Image.fromarray(map_array)
    image.save(pgm_filename)
    print("\nSAVE MAP: {}".format(pgm_filename))
    
    yaml_data = {
        'image': pgm_filename,
        'resolution': float(resolution),
        'origin': [-world_width/2, -world_height/2, 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.196
    }
    
    yaml_filename = "{}.yaml".format(map_name)
    with open(yaml_filename, 'w') as f:
        yaml.dump(yaml_data, f, default_flow_style=False)
    
    print("SAVE: {}".format(yaml_filename))
    print("\nMAP INFO:")
    print("  SIZE: PIXEL{}x{}".format(map_array.shape[1], map_array.shape[0]))
    print("  RATIO: {} m/pixel".format(resolution))
    print("  SIZE: {}x{} m".format(world_width, world_height))
    print("  ORIGIN: ({:.1f}, {:.1f})".format(yaml_data['origin'][0], yaml_data['origin'][1]))
    
    return pgm_filename, yaml_filename

def main():
    print("FROM Gazebo world...")
    
    map_array, resolution, world_width, world_height = create_map_from_world()
    
    pgm_file, yaml_file = save_map(map_array, resolution, world_width, world_height)
    
    print("\nSUCCESS:")
    print("  MAP: {}".format(pgm_file))
    print("  CONFIG: {}".format(yaml_file))
    print("\nMANNER:")
    print("  rosrun map_server map_server {}".format(yaml_file))

if __name__ == "__main__":
    main()
