import cv2
from PIL import Image
import numpy as np
import rasterio
import rasterio.crs
from rasterio.transform import from_origin

# Load images with PIL
img_path_pil = Image.open("path.png").convert("L")
img_obstacle_pil = Image.open("obstacle.png").convert("L")

# Convert pil images to numpy arrays
img_path = np.array(img_path_pil)
img_obstacle = np.array(img_obstacle_pil)
assert img_path.shape == img_obstacle.shape, "Input images must have the same dimensions"

transform = from_origin(-10, -10, 1, 1)
crs = rasterio.crs.CRS.from_epsg(4326)

# Create a new raster file with the same dimensions as the input images and two bands
with rasterio.open(
    "output.tiff", "w",
    driver="GTiff",
    width=img_path.shape[1],
    height=img_path.shape[0],
    count=2,
    dtype=img_path.dtype,
    transform=transform,
) as dst:
    dst.write(img_path, 1)  # Write the path image as band 1
    dst.write(img_obstacle, 2)  # Write the obstacle image as band 2

print("GeoTIFF file saved successfully as 'output.tiff'")
