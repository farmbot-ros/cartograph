#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include "farmbot_interfaces/msg/geo_tiff.hpp"
#include "farmbot_interfaces/srv/gps2_enu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "spdlog/spdlog.h"

using namespace std::chrono_literals;
namespace echo = spdlog;

class GeoPolygonTimerNode : public rclcpp::Node {
private:
    rclcpp::Subscription<farmbot_interfaces::msg::GeoTiff>::SharedPtr geotiff_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Client<farmbot_interfaces::srv::Gps2Enu>::SharedPtr gps2enu_client_;
    rclcpp::Publisher<farmbot_interfaces::msg::GeoTiff>::SharedPtr geotiff_enu_pub_;

    farmbot_interfaces::msg::GeoTiff geotiff_gnss_;
    farmbot_interfaces::msg::GeoTiff geotiff_enu_;

public:
    GeoPolygonTimerNode() : Node("vectpub") {
;
    }

private:
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GeoPolygonTimerNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
