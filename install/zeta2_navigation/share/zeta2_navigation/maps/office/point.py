import yaml
from PIL import Image

def load_map_data(pgm_file, yaml_file):
    # Load the image (PGM file)
    image = Image.open(pgm_file)
    width, height = image.size

    # Load the map metadata (YAML file)
    with open(yaml_file, 'r') as file:
        map_metadata = yaml.safe_load(file)
    
    resolution = map_metadata['resolution']
    origin = map_metadata['origin']

    return width, height, resolution, origin

def pixel_to_world(u, v, width, height, resolution, origin):
    # Convert pixel coordinates to world coordinates
    x = u * resolution + origin[0]
    y = (height - v) * resolution + origin[1]
    return x, y

# Example usage
pgm_file = 'map1.pgm'
yaml_file = 'mapping.yaml'

width, height, resolution, origin = load_map_data(pgm_file, yaml_file)
u, v = 167, 92  # Example pixel coordinates
x, y = pixel_to_world(u, v, width, height, resolution, origin)

print(f"Pixel coordinates ({u}, {v}) -> World coordinates ({x}, {y})")
