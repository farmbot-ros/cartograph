import cv2
from PIL import Image
import numpy as np
import rasterio
import rasterio.crs
from rasterio.transform import from_origin

# Load images with PIL
img_path_pil = Image.open("path.png").convert("L")
img_obstacle_pil = Image.open("obstacle.png").convert("L")

# Convert PIL images to arrays compatible with OpenCV (BGR instead of RGBA for display)
img_path_cv2 = cv2.cvtColor(np.array(img_path_pil), cv2.COLOR_RGBA2BGR)
img_obstacle_cv2 = cv2.cvtColor(np.array(img_obstacle_pil), cv2.COLOR_RGBA2BGR)

# Show images in separate windows
cv2.imshow("Path", img_path_cv2)
cv2.imshow("Obstacle", img_obstacle_cv2)
while True:
    if cv2.waitKey(1) & 0xFF == ord('q'): break
cv2.destroyAllWindows()

# Convert pil images to numpy arrays
img_path = np.array(img_path_pil)
img_obstacle = np.array(img_obstacle_pil)
assert img_path.shape == img_obstacle.shape, "Input images must have the same dimensions"

transform = from_origin(-10, -10, 1, 1)
crs = rasterio.crs.CRS.from_epsg(4326)

metadata = {
    "description": "GeoTIFF containing path and obstacle data",
    "author": "Your Name",
    "date": "2024-12-25",
    "crs": "EPSG:4326",  # Coordinate Reference System (example)
}

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
    dst.update_tags(**metadata)
    dst.update_tags(a='1', b='2')

print("GeoTIFF file saved successfully as 'output.tiff'")


# import tifffile as tiff
# with tiff.TiffFile("output.tiff") as tif:
#     print(tif.pages[0].tags)
#     print(tif.pages[1].tags)