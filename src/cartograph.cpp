#include <rclcpp/rclcpp.hpp>
#include <string>
#include "geospatial_interfaces/msg/geo_json.hpp"
#include "geospatial_interfaces/msg/geo_tiff.hpp"
#include "farmbot_cartograph/geojson.hpp"
#include "farmbot_cartograph/geotiff.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <spdlog/spdlog.h>

namespace echo = spdlog;

class GeoParserNode {
private:
    rclcpp::Node::SharedPtr node;
    std::string geojson_file_path;
    std::string geotiff_file_path;
    bool inited = false;

    rclcpp::TimerBase::SharedPtr timer_;
    //geogjon publisher
    rclcpp::Publisher<geospatial_interfaces::msg::GeoJson>::SharedPtr geojson_publisher;
    geospatial_interfaces::msg::GeoJson geojson_msg;
    //geotiff publisher
    rclcpp::Publisher<geospatial_interfaces::msg::GeoTiff>::SharedPtr geotiff_publisher;
    geospatial_interfaces::msg::GeoTiff geotiff_msg;

public:
    GeoParserNode(const rclcpp::Node::SharedPtr &node) : node(node) {
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("farmbot_cartograph");
        geojson_file_path = package_share_directory + "/config/wur.json";
        geotiff_file_path = package_share_directory + "/config/wur.tiff";

        //timer publisher
        timer_ = node->create_wall_timer(std::chrono::seconds(1), std::bind(&GeoParserNode::timer_callback, this));

        //geogjon publisher
        geojson_publisher = node->create_publisher<geospatial_interfaces::msg::GeoJson>("geojson", 10);
        //geotiff publisher
        geotiff_publisher = node->create_publisher<geospatial_interfaces::msg::GeoTiff>("geotiff", 10);

        // Parse GeoJSON
        try {
            geojson_msg = geojson_parser::parseGeoJson(geojson_file_path);
            RCLCPP_INFO(node->get_logger(), "Parsed GeoJSON with %d features", geojson_msg.num_features);
            for (size_t i = 0; i < geojson_msg.features.size(); ++i) {
                RCLCPP_INFO(node->get_logger(), "Feature %zu has %zu property entries", i, geojson_msg.features[i].properties.size());
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR(node->get_logger(), "Error parsing GeoJSON: %s", e.what());
        }

        // // Parse GeoTIFF
        try {
            geotiff_msg = geotiff_parser::parseGeoTiff(geotiff_file_path);
            RCLCPP_INFO(node->get_logger(), "Parsed GeoTIFF with %d band(s)", geotiff_msg.num_bands);
            for (size_t i = 0; i < geotiff_msg.bands.size(); ++i) {
                RCLCPP_INFO(node->get_logger(), "Band %zu: data length %zu, properties %zu",
                            i,
                            geotiff_msg.bands[i].data.size(),
                            geotiff_msg.bands[i].properties.size());
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR(node->get_logger(), "Error parsing GeoTIFF: %s", e.what());
        }
        inited = true;
    }

    void timer_callback() {
        if (!inited) {
            return;
        }
        geojson_publisher->publish(geojson_msg);
        echo::info("Published GeoJSON");
        geotiff_publisher->publish(geotiff_msg);
        echo::info("Published GeoTIFF");
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto main_node = std::make_shared<rclcpp::Node>("geo_parser_node");
    auto geo_parser = std::make_shared<GeoParserNode>(main_node);

    rclcpp::spin(main_node);
    rclcpp::shutdown();
    return 0;
}
