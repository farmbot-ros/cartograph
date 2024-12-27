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

    std::vector<geometry_msgs::msg::Polygon> polygons_enu_;
    std::vector<geometry_msgs::msg::Polygon> polygons_gnss_;
    std::vector<rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr> polygon_publishers_;

    rclcpp::Client<farmbot_interfaces::srv::Gps2Enu>::SharedPtr gps2enu_client_;

public:
    GeoPolygonTimerNode() : Node("geo_polygon_timer_node") {
        geojson_sub_ = this->create_subscription<farmbot_interfaces::msg::GeoJson>(
            "geojson",
            10,
            std::bind(&GeoPolygonTimerNode::geoJsonCallback, this, std::placeholders::_1)
        );

        gps2enu_client_ = this->create_client<farmbot_interfaces::srv::Gps2Enu>(
            "loc/gps2enu",
            rmw_qos_profile_services_default
        );

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&GeoPolygonTimerNode::timerCallback, this)
        );
    }

private:
    void geoJsonCallback(const farmbot_interfaces::msg::GeoJson::SharedPtr msg) {
        polygon_publishers_.clear();
        polygons_gnss_.clear();

        size_t polygon_index = 0;
        for (const auto &feature : msg->features) {
            if (feature.geometry.type == farmbot_interfaces::msg::Geometry::POLYGON) {
                geometry_msgs::msg::Polygon polygon_msg;
                for (const auto &coord : feature.geometry.cordinates.points) {
                    geometry_msgs::msg::Point32 pt;
                    pt.x = static_cast<float>(coord.x);
                    pt.y = static_cast<float>(coord.y);
                    pt.z = static_cast<float>(coord.z);
                    polygon_msg.points.push_back(pt);
                }
                std::string topic_name = "polygon_" + std::to_string(polygon_index);
                auto pub = this->create_publisher<geometry_msgs::msg::Polygon>(topic_name, 10);
                polygon_publishers_.push_back(pub);
                polygons_gnss_.push_back(polygon_msg);
                polygon_index++;
            }
        }
        geojson_sub_.reset();
    }

    void timerCallback() {
        // Only kick off nav_to_enu once after we get polygons from GeoJSON.
        if (!trying_to_convert_) {
            trying_to_convert_ = true;
            nav_to_enu(polygons_gnss_);
        }

        // Publish what we have in polygons_enu_.
        for (size_t i = 0; i < polygons_enu_.size(); i++) {
            polygon_publishers_[i]->publish(polygons_enu_[i]);
        }
    }

    // Send each polygon to the service asynchronously.
    void nav_to_enu(const std::vector<geometry_msgs::msg::Polygon>& navpts) {
        if (!gps2enu_client_->service_is_ready()) {
            RCLCPP_WARN(this->get_logger(), "GPS2ENU service not ready.");
            trying_to_convert_ = false;
            return;
        }

        // Prepare the ENU polygons vector to match size
        polygons_enu_.resize(navpts.size());

        for (size_t i = 0; i < navpts.size(); i++) {
            auto request = std::make_shared<farmbot_interfaces::srv::Gps2Enu::Request>();
            for (auto &point : navpts[i].points) {
                sensor_msgs::msg::NavSatFix gps_point;
                gps_point.latitude = point.y;
                gps_point.longitude = point.x;
                gps_point.altitude = point.z;
                request->gps.push_back(gps_point);
            }

            // For each polygon, we send a separate request; the callback updates polygons_enu_ at index i.
            auto response_callback =
                [this, i](rclcpp::Client<farmbot_interfaces::srv::Gps2Enu>::SharedFuture future) {
                    auto result = future.get();
                    if (!result) {
                        RCLCPP_ERROR(this->get_logger(), "GPS2ENU service call failed.");
                        return;
                    }
                    geometry_msgs::msg::Polygon enu_polygon;
                    for (auto &point : result->enu) {
                        geometry_msgs::msg::Point32 enu_point;
                        enu_point.x = point.position.x;
                        enu_point.y = point.position.y;
                        enu_point.z = point.position.z;
                        enu_polygon.points.push_back(enu_point);
                    }
                    polygons_enu_[i] = enu_polygon;

                    // If this is the last polygon to get updated, we can reset trying_to_convert_.
                    // (Alternatively, you could count how many are completed, etc.)
                    if (i == polygons_enu_.size() - 1) {
                        echo::info("All polygons converted to ENU.");
                    }
                };

            gps2enu_client_->async_send_request(request, response_callback);
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
