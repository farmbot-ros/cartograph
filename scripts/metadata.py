import rasterio

# Path to the TIFF file
file_path = "../config/wur.tiff"

# Open the TIFF file in 'r+' mode to edit metadata
with rasterio.open(file_path, mode='r+') as src:
    # Update global metadata
    # src.update_tags()
    src.update_tags(**{})
    # Add metadata to Band 1
    src.update_tags(1, type="path")
    # Add metadata to Band 2
    src.update_tags(2, type="obstacle")



with rasterio.open(file_path) as src:
    # Print general metadata
    print("=== General Metadata ===")
    print("Driver:", src.driver)
    print("Width:", src.width)
    print("Height:", src.height)
    print("Count (Number of Bands):", src.count)
    print("Dtype (Data Type):", src.dtypes)
    print("CRS (Coordinate Reference System):", src.crs)
    print("Transform:", src.transform)
    print("Bounds:", src.bounds)
    print("Metadata:", src.meta)
    print()

    # Print tags (global dataset tags)
    print("=== Global Tags ===")
    print(src.tags())
    print()

    # Print tags for each band
    for band in range(1, src.count + 1):
        print(f"=== Band {band} Tags ===")
        print(src.tags(band))
        print()

    # Print pixel values (optional, for small datasets)
    print("=== Pixel Values (First 5x5 Window) ===")
    for band in range(1, src.count + 1):
        data = src.read(band, window=((0, 5), (0, 5)))  # Adjust size as needed
        print(f"Band {band} Values:")
        print(data)
        print()
