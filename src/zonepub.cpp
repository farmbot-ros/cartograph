#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include "farmbot_interfaces/msg/geo_json.hpp"
#include "farmbot_interfaces/msg/feature.hpp"
#include "farmbot_interfaces/msg/geometry.hpp"
#include "farmbot_interfaces/msg/cordinates.hpp"
#include "farmbot_interfaces/msg/cordinate.hpp"
#include "farmbot_interfaces/srv/gps2_enu.hpp"
#include "spdlog/spdlog.h"

using namespace std::chrono_literals;
namespace echo = spdlog;

class GeoPolygonTimerNode : public rclcpp::Node {
    private:
        rclcpp::Subscription<farmbot_interfaces::msg::GeoJson>::SharedPtr geojson_sub_;
        rclcpp::TimerBase::SharedPtr timer_;
        bool trying_to_convert_ = false;

        // Store each polygon along with its publisher.
        std::vector<geometry_msgs::msg::Polygon> polygons_enu_;
        std::vector<geometry_msgs::msg::Polygon> polygons_gnss_;
        std::vector<rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr> polygon_publishers_;

        // Client to convert GNSS to ENU coordinates.
        rclcpp::Client<farmbot_interfaces::srv::Gps2Enu>::SharedPtr gps2enu_client_;

    public:
        GeoPolygonTimerNode() : Node("geo_polygon_timer_node") {
            // Subscribe to GeoJson. After receiving the first message, we'll close this subscription.
            geojson_sub_ = this->create_subscription<farmbot_interfaces::msg::GeoJson>(
                "geojson",
                10,
                std::bind(&GeoPolygonTimerNode::geoJsonCallback, this, std::placeholders::_1)
            );

            // Create the GPS to ENU client, assign it to the client callback group
            gps2enu_client_ = this->create_client<farmbot_interfaces::srv::Gps2Enu>(
                "loc/gps2enu",
                rmw_qos_profile_services_default);

            // Create a timer to publish polygons continuously.
            // Adjust the publishing rate to your preference (currently 1 second).
            timer_ = this->create_wall_timer(
                std::chrono::seconds(1),
                std::bind(&GeoPolygonTimerNode::timerCallback, this)
            );
        }

    private:
        void geoJsonCallback(const farmbot_interfaces::msg::GeoJson::SharedPtr msg) {
            // Clear any previously stored polygons/publishers (if any).
            polygon_publishers_.clear();
            polygons_gnss_.clear();

            size_t polygon_index = 0;
            for (const auto &feature : msg->features) {
                // Only handle features of type POLYGON.
                if (feature.geometry.type == farmbot_interfaces::msg::Geometry::POLYGON) {
                    geometry_msgs::msg::Polygon polygon_msg;
                    // Convert each coordinate to Point32 for ROS Polygon.
                    for (const auto &coord : feature.geometry.cordinates.points) {
                        geometry_msgs::msg::Point32 pt;
                        pt.x = static_cast<float>(coord.x);
                        pt.y = static_cast<float>(coord.y);
                        pt.z = static_cast<float>(coord.z);
                        polygon_msg.points.push_back(pt);
                    }

                    // Dynamically create a publisher for this polygon.
                    std::string topic_name = "polygon_" + std::to_string(polygon_index);
                    auto pub = this->create_publisher<geometry_msgs::msg::Polygon>(topic_name, 10);

                    // Store the publisher and the Polygon message.
                    polygon_publishers_.push_back(pub);
                    polygons_gnss_.push_back(polygon_msg);

                    polygon_index++;
                }
            }
            // Close (reset) the subscription so we only parse once.
            geojson_sub_.reset();
        }

        void timerCallback() {
            if (!trying_to_convert_) {
                nav_to_enu(polygons_gnss_);
                trying_to_convert_ = true;
                echo::info("Trying to convert GNSS to ENU...");
            }
            // Continuously publish all the polygons (if any).
            for (size_t i = 0; i < polygons_enu_.size(); i++) {
                polygon_publishers_[i]->publish(polygons_enu_[i]);
            }
        }

        void nav_to_enu(const std::vector<geometry_msgs::msg::Polygon>& navpts) {
            std::vector<geometry_msgs::msg::Polygon> polygons;
            auto request = std::make_shared<farmbot_interfaces::srv::Gps2Enu::Request>();
            for (const auto& polygon : navpts) {
                geometry_msgs::msg::Polygon enu_polygon;
                for (const auto& point : polygon.points) {
                    sensor_msgs::msg::NavSatFix gps_point;
                    gps_point.latitude = point.x;
                    gps_point.longitude = point.y;
                    gps_point.altitude = point.z;  // Adjust if altitude data is available
                    request->gps.push_back(gps_point);
                }
                while (!gps2enu_client_->wait_for_service(1s)) {
                    if (!rclcpp::ok()) {
                        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                    }
                    RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
                }
                echo::info("Service available, sending request...");
                // Send the request and wait for the result (blocking call)
                auto result_future = gps2enu_client_->async_send_request(request);
                while (rclcpp::ok() && result_future.wait_for(1s) == std::future_status::timeout) {
                    RCLCPP_INFO(this->get_logger(), "Waiting for response from GPS2ENU service...");
                }
                echo::info("Received response from GPS2ENU service.");
                auto result = result_future.get();
                if (!result) {
                    RCLCPP_ERROR(this->get_logger(), "Service call failed.");
                }
                echo::info("Service call successful.");
                auto getres = result->enu;
                for (const auto& point : getres) {
                    geometry_msgs::msg::Point32 enu_point;
                    enu_point.x = point.position.x;
                    enu_point.y = point.position.y;
                    enu_point.z = point.position.z;
                    enu_polygon.points.push_back(enu_point);
                }
                polygons.push_back(enu_polygon);
            }
            polygons_enu_ = polygons;
        }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GeoPolygonTimerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
