#include <farmbot_interfaces/msg/detail/geo_json__struct.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include "farmbot_interfaces/msg/geo_json.hpp"
#include "farmbot_interfaces/msg/feature.hpp"
#include "farmbot_interfaces/msg/geometry.hpp"
#include "farmbot_interfaces/msg/cordinates.hpp"
#include "farmbot_interfaces/msg/cordinate.hpp"
#include "farmbot_interfaces/srv/gps2_enu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "spdlog/spdlog.h"

using namespace std::chrono_literals;
namespace echo = spdlog;

class GeoPolygonTimerNode : public rclcpp::Node {
private:
    rclcpp::Subscription<farmbot_interfaces::msg::GeoJson>::SharedPtr geojson_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool trying_to_convert_ = false;
    bool converted_ = false;

    rclcpp::Client<farmbot_interfaces::srv::Gps2Enu>::SharedPtr gps2enu_client_;
    rclcpp::Publisher<farmbot_interfaces::msg::GeoJson>::SharedPtr geojson_enu_pub_;

    farmbot_interfaces::msg::GeoJson geojson_gnss_;
    farmbot_interfaces::msg::GeoJson geojson_enu_;

public:
    GeoPolygonTimerNode() : Node("geo_polygon_timer_node") {
        geojson_sub_ = this->create_subscription<farmbot_interfaces::msg::GeoJson>("/map/geojson", 10,
            [this](const farmbot_interfaces::msg::GeoJson::SharedPtr msg) {
                geojson_gnss_ = *msg;
            });

        gps2enu_client_ = this->create_client<farmbot_interfaces::srv::Gps2Enu>("loc/gps2enu");
        geojson_enu_pub_ = this->create_publisher<farmbot_interfaces::msg::GeoJson>("/map/geojson/enu", 10);

        timer_ = this->create_wall_timer(
            1s, std::bind(&GeoPolygonTimerNode::timerCallback, this));
    }

private:
    void timerCallback() {
        if (!trying_to_convert_) {
            trying_to_convert_ = true;
            RCLCPP_INFO(this->get_logger(), "Starting GPS to ENU conversion.");
            // nav_to_enu(geojson_gnss_);
            //put in a thread
            std::thread(&GeoPolygonTimerNode::nav_to_enu, this, geojson_gnss_).detach();
        }
        if (converted_) {
            geojson_enu_pub_->publish(geojson_enu_);
        }
    }

    void nav_to_enu(const farmbot_interfaces::msg::GeoJson& geojson) {
        if (!gps2enu_client_->service_is_ready()) {
            RCLCPP_WARN(this->get_logger(), "GPS2ENU service not ready.");
            trying_to_convert_ = false;
            return;
        }
        geojson_enu_ = geojson_gnss_;
        for (size_t i = 0; i < geojson.features.size(); ++i) {
            const auto& feature = geojson.features[i];
            auto request = std::make_shared<farmbot_interfaces::srv::Gps2Enu::Request>();

            for (const auto& cord : feature.geometry.cordinates.points) {
                sensor_msgs::msg::NavSatFix gps_point;
                gps_point.latitude = cord.y;
                gps_point.longitude = cord.x;
                gps_point.altitude = cord.z;
                request->gps.push_back(gps_point);
            }
            echo::info("Sending request to GPS2ENU service.");

            gps2enu_client_->async_send_request(request,
                [this, i, geojson](rclcpp::Client<farmbot_interfaces::srv::Gps2Enu>::SharedFuture future) {
                    try {
                        auto result = future.get();
                        geojson_enu_.features[i].geometry.cordinates.points.clear();

                        for (const auto& point : result->enu) {
                            farmbot_interfaces::msg::Cordinate enu_cord;
                            enu_cord.x = point.position.x;
                            enu_cord.y = point.position.y;
                            enu_cord.z = point.position.z;
                            geojson_enu_.features[i].geometry.cordinates.points.push_back(enu_cord);
                        }

                        if (i == geojson.features.size() - 1) {
                            RCLCPP_INFO(this->get_logger(), "All polygons converted to ENU.");
                            converted_ = true;
                        }
                    } catch (const std::exception& e) {
                        RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
                    }
                }
            );
        }
    }
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
