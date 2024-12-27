#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include "farmbot_interfaces/msg/geo_json.hpp"
#include "farmbot_interfaces/msg/feature.hpp"
#include "farmbot_interfaces/msg/geometry.hpp"
#include "farmbot_interfaces/msg/cordinates.hpp"
#include "farmbot_interfaces/msg/cordinate.hpp"

class GeoPolygonTimerNode : public rclcpp::Node {
    public:
        GeoPolygonTimerNode() : Node("geo_polygon_timer_node") {
            // Subscribe to GeoJson. After receiving the first message, we'll close this subscription.
            geojson_sub_ = this->create_subscription<farmbot_interfaces::msg::GeoJson>(
                "geojson_in",
                10,
                std::bind(&GeoPolygonTimerNode::geoJsonCallback, this, std::placeholders::_1)
            );

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
            polygons_.clear();

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
                    polygons_.push_back(polygon_msg);

                    polygon_index++;
                }
            }

            RCLCPP_INFO(
                this->get_logger(),
                "Received GeoJson with %zu polygon(s). Closing subscription.",
                polygon_publishers_.size()
            );

            // Close (reset) the subscription so we only parse once.
            geojson_sub_.reset();
        }

        void timerCallback() {
            // Continuously publish all the polygons (if any).
            for (size_t i = 0; i < polygons_.size(); i++) {
                polygon_publishers_[i]->publish(polygons_[i]);
            }
        }

        rclcpp::Subscription<farmbot_interfaces::msg::GeoJson>::SharedPtr geojson_sub_;
        rclcpp::TimerBase::SharedPtr timer_;

        // Store each polygon along with its publisher.
        std::vector<geometry_msgs::msg::Polygon> polygons_;
        std::vector<rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr> polygon_publishers_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GeoPolygonTimerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
