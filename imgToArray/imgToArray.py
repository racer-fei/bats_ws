from PIL import Image
import numpy as np

def image_to_2d_array(image_path):
    """
    Converts an image to a 2D array for use with Nav2's A* algorithm.
    
    Parameters:
        image_path (str): Path to the image file.
    
    Returns:
        np.ndarray: 2D array representing the map.
    """
    # Load
    img = Image.open(image_path)

    # Resize
    img = img.resize((200, 200), Image.NEAREST)

    img_array = np.array(img)

    map_array = np.zeros((img_array.shape[0], img_array.shape[1]), dtype='<U2')

    #Analise pixel por pixel
    for y in range(img_array.shape[0]):
        for x in range(img_array.shape[1]):
            pixel = img_array[y, x]
            # Pixel puramente verde -> Start
            if isinstance(pixel, np.ndarray) and np.all(pixel[:3] == [0, 255, 0]):
                map_array[y, x] = 'S'
            # Pixel branco = Livre
            elif isinstance(pixel, np.ndarray) and np.all(pixel[:3] == [255, 255, 255]):
                map_array[y, x] = '0'
            # Pixel de qualquer outra cor -> Obstáculo
            else:
                map_array[y, x] = '1'

    return map_array

def save_array_to_file(array, file_path):
    """
    Saves a 2D array to a text file.
    
    Parameters:
        array (np.ndarray): 2D array to save.
        file_path (str): Path to the output file.
    """
    np.savetxt(file_path, array, fmt='%s')

if __name__ == "__main__":
    # Path to the input image
    input_image_path = 'test.png'
    
    # Convert the image to a 2D array
    map_array = image_to_2d_array(input_image_path)
    
    # Save the 2D array to a file
    output_file_path = '2DArray.txt'
    save_array_to_file(map_array, output_file_path)
    
    print("Sucesso, salvo como->", output_file_path)
