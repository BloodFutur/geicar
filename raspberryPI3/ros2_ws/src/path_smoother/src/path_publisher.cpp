#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <fstream>
#include <sstream>
#include <vector>
#include <string>

class PathPublisher : public rclcpp::Node {
public:
    PathPublisher()
    : Node("path_publisher") {
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("path", 10);
        std::string file_path = "path.csv";
        loadPathData(file_path);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&PathPublisher::publishPath, this));
    }

private:
    void loadPathData(const std::string& file_path) {
        std::ifstream file(file_path);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", file_path.c_str());
            return;
        }

        std::string line;
        std::getline(file, line); // Skip the header

        int pose_count = 0; // Add a counter for poses

        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string data;
            std::vector<std::string> row_data;

            while (std::getline(ss, data, ',')) {
                row_data.push_back(data);
            }

            geometry_msgs::msg::PoseStamped pose;
            pose.header.stamp = this->now();
            pose.header.frame_id = "odom";
            pose.pose.position.x = std::stod(row_data[0]);
            pose.pose.position.y = std::stod(row_data[1]);
            pose.pose.position.z = 0;
            pose.pose.orientation.x = 0;
            pose.pose.orientation.y = 0;
            pose.pose.orientation.z = std::stod(row_data[2]);
            pose.pose.orientation.w = std::stod(row_data[3]);
            path_.poses.push_back(pose);
            pose_count++; // Increment the counter
        }

        path_.header.stamp = this->now();
        path_.header.frame_id = "odom";

        RCLCPP_INFO(this->get_logger(), "Loaded %d poses from the CSV file.", pose_count); // Log the number of loaded poses
    }

    void publishPath() {
        path_.header.stamp = this->now();
        path_.header.frame_id = "odom";
        path_pub_->publish(path_);
    }

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    nav_msgs::msg::Path path_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
