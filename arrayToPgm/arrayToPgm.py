import numpy as np
from PIL import Image

def load_array(file_path):
    return np.loadtxt(file_path, dtype=str)

def array_to_pgm(array, pgm_path):
    height, width = array.shape
    img = Image.new('L', (width, height))
    pixels = img.load()

    for i in range(width):
        for j in range(height):
            if array[j, i] == '1':  # Obstaculo = Preto
                pixels[i, j] = 0  
            elif array[j, i] == '0':  
                pixels[i, j] = 255  # Livre = Branco
            elif array[j, i] == 'S': 
                pixels[i, j] = 128  # Início = Cinza

    img.save(pgm_path)

def create_yaml(pgm_path, yaml_path):
    yaml_content = f"""
image: {pgm_path}
resolution: 1.0
origin: [0.0, 0.0, 0.0]
occupied_thresh: 0.65
free_thresh: 0.196
negate: 0
"""
    with open(yaml_path, 'w') as yaml_file:
        yaml_file.write(yaml_content)

if __name__ == "__main__":
    array_path = '2DArray.txt'
    pgm_path = 'map.pgm'
    yaml_path = 'map.yaml'

    array = load_array(array_path)
    array_to_pgm(array, pgm_path)
    create_yaml(pgm_path, yaml_path)
    
    print(f"PGM salvo em: {pgm_path}")
    print(f"YAML salvo em: {yaml_path}")

