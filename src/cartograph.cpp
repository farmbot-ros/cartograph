#include <rclcpp/rclcpp.hpp>
#include <string>
#include "farmbot_interfaces/msg/geo_json.hpp"
#include "farmbot_interfaces/msg/geo_tiff.hpp"
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
    rclcpp::Publisher<farmbot_interfaces::msg::GeoJson>::SharedPtr geojson_publisher;
    farmbot_interfaces::msg::GeoJson geojson_msg;
    //geotiff publisher
    rclcpp::Publisher<farmbot_interfaces::msg::GeoTiff>::SharedPtr geotiff_publisher;
    farmbot_interfaces::msg::GeoTiff geotiff_msg;

public:
    GeoParserNode(const rclcpp::Node::SharedPtr &node) : node(node) {
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("farmbot_cartograph");
        geojson_file_path = package_share_directory + "/config/wur.json";
        geotiff_file_path = package_share_directory + "/config/wur.tiff";

        //timer publisher
        timer_ = node->create_wall_timer(std::chrono::seconds(1), std::bind(&GeoParserNode::timer_callback, this));

        // GeoJSON
        geojson_publisher = node->create_publisher<farmbot_interfaces::msg::GeoJson>("geojson", 10);
        try {
            geojson_msg = geojson_parser::parseGeoJson(geojson_file_path);
            RCLCPP_INFO(node->get_logger(), "Parsed GeoJSON with %d features", geojson_msg.num_features);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(node->get_logger(), "Error parsing GeoJSON: %s", e.what());
        }

        // /GeoTIFF
        geotiff_publisher = node->create_publisher<farmbot_interfaces::msg::GeoTiff>("geotiff", 10);
        try {
            geotiff_msg = geotiff_parser::parseGeoTiff(geotiff_file_path);
            RCLCPP_INFO(node->get_logger(), "Parsed GeoTIFF with %d band(s)", geotiff_msg.num_bands);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(node->get_logger(), "Error parsing GeoTIFF: %s", e.what());
        }

        // Set inited flag
        inited = true;
    }

    void timer_callback() {
        if (!inited) { return; }
        geojson_publisher->publish(geojson_msg);
        geotiff_publisher->publish(geotiff_msg);
        echo::info("Published GeoJSON and GeoTIFF");
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    //node options
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);
    // Create node
    auto main_node = std::make_shared<rclcpp::Node>("geoparser_node", options);
    // Create GeoParserNode
    auto geo_parser = std::make_shared<GeoParserNode>(main_node);

    rclcpp::spin(main_node);
    rclcpp::shutdown();
    return 0;
}
