#include <gdal_priv.h>
#include <ogr_spatialref.h>
#include <cpl_conv.h> // CPLFree
#include <stdexcept>
#include <string>
#include <vector>

#include "farmbot_interfaces/msg/geo_tiff.hpp"
#include "farmbot_interfaces/msg/band.hpp"
#include "farmbot_interfaces/msg/grid.hpp"
#include "farmbot_interfaces/msg/propertie.hpp"

namespace geotiff_parser{
    using farmbot_interfaces::msg::GeoTiff;
    using farmbot_interfaces::msg::Band;
    using farmbot_interfaces::msg::Grid;
    using farmbot_interfaces::msg::Propertie;

    GeoTiff parseGeoTiff(const std::string &file_path) {
        // Register all GDAL drivers
        GDALAllRegister();

        // Open the dataset in read-only mode
        GDALDataset *dataset = (GDALDataset *) GDALOpen(file_path.c_str(), GA_ReadOnly);
        if (!dataset) {
            throw std::runtime_error("Failed to open GeoTIFF: " + file_path);
        }

        GeoTiff geo_tiff_msg;
        geo_tiff_msg.num_bands = 0; // Will fill below

        // Projection
        const char *proj_str = dataset->GetProjectionRef();
        if (proj_str) {
            geo_tiff_msg.projection = proj_str;
        }

        // Origin (we'll store top-left corner in Pose.position)
        // Also read resolution from the GeoTransform
        double geo_transform[6] = {0};
        if (dataset->GetGeoTransform(geo_transform) == CE_None) {
            geo_tiff_msg.origin.position.x = geo_transform[0]; // top-left corner X
            geo_tiff_msg.origin.position.y = geo_transform[3]; // top-left corner Y
            // For simplicity, leave Z=0, orientation as identity
        }

        // Number of raster bands
        int band_count = dataset->GetRasterCount();
        geo_tiff_msg.num_bands = band_count;

        // Read each band
        for (int b = 1; b <= band_count; ++b) {
            GDALRasterBand *gdal_band = dataset->GetRasterBand(b);
            if (!gdal_band) {
                continue;
            }

            Band band_msg;

            // We store a single Grid object in this example (index 0 of band_msg.grid[])
            Grid grid;
            // The pixel resolution is typically geo_transform[1] (X) and geo_transform[5] (Y)
            // We'll store a simple absolute resolution in grid.resolution
            double pixel_res_x = geo_transform[1];
            double pixel_res_y = geo_transform[5];
            // If pixel_res_y is negative, make it positive for the "resolution" field
            if (pixel_res_y > 0) {
                pixel_res_y = -pixel_res_y;
            }
            // You can choose how to interpret resolution. Here we just store the X resolution:
            grid.resolution = static_cast<float>(pixel_res_x);

            // Width (columns) & height (rows)
            grid.width = gdal_band->GetXSize();
            grid.height = gdal_band->GetYSize();

            band_msg.grid.push_back(grid);

            // Read properties / metadata from the band
            // For example, we can fetch "STATISTICS_MINIMUM", etc.
            char **metadata = gdal_band->GetMetadata();
            if (metadata) {
                for (int i = 0; metadata[i] != nullptr; ++i) {
                    Propertie prop;
                    std::string meta_entry(metadata[i]);
                    // The format of meta_entry is KEY=VALUE
                    auto eq_pos = meta_entry.find('=');
                    if (eq_pos != std::string::npos) {
                        prop.key = meta_entry.substr(0, eq_pos);
                        prop.value = meta_entry.substr(eq_pos + 1);
                    } else {
                        prop.key = "Metadata"; // fallback
                        prop.value = meta_entry;
                    }
                    band_msg.properties.push_back(prop);
                }
            }

            // Read the actual data as int16
            // Make sure the band type is GDT_Int16 or cast it
            int width = gdal_band->GetXSize();
            int height = gdal_band->GetYSize();
            band_msg.data.resize(width * height);
            CPLErr err = gdal_band->RasterIO(
                GF_Read,
                0,                      // startX
                0,                      // startY
                width,                  // width
                height,                 // height
                band_msg.data.data(),   // destination buffer
                width,
                height,
                GDT_Int16,
                0, 0                    // pixel and line spacing
            );
            if (err != CE_None) {
                throw std::runtime_error("Failed to read raster band data.");
            }

            // Add this band to the GeoTiff message
            geo_tiff_msg.bands.push_back(band_msg);
        }

        GDALClose(dataset);
        return geo_tiff_msg;
    }
} // namespace geotiff_parser
