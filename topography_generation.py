import pdr
import numpy as np
from PIL import Image

topography = pdr.read("megt90n000eb.lbl")
coords = np.array(topography['IMAGE'])
shift = coords.shape[1] // 2
transformed_coords = np.roll(coords, shift, axis = 1)

pixel_data = 255*((transformed_coords - np.min(transformed_coords)) / (np.max(transformed_coords) - np.min(transformed_coords)))
pixel_int = np.round(pixel_data).astype(np.uint8)
img = Image.fromarray(pixel_int)
img.save("mars.png")
