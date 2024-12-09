#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

// Class definition for the IMU Frame ID Modifier Node
class ImuFrameIdModifier : public rclcpp::Node {
public:
    ImuFrameIdModifier() : Node("imu_frame_id_modifier") {
        // Subscription to the input topic (/imu/data)
        subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10,
            std::bind(&ImuFrameIdModifier::imu_callback, this, std::placeholders::_1));

        // Publisher for the modified topic (/imu/modified_data)
        publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/modified_frame_id", 10);
    }

private:
    // Callback function for the subscription
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        // Create a copy of the received message
        auto modified_msg = *msg;

        // Modify the frame_id of the header
        modified_msg.header.frame_id = "base_link"; // Replace with desired frame_id

        // Publish the modified message
        publisher_->publish(modified_msg);
    }

    // Subscription object
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;

    // Publisher object
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
};

// Main function
int main(int argc, char **argv) {
    // Initialize the ROS 2 system
    rclcpp::init(argc, argv);

    // Spin the node (process callbacks)
    rclcpp::spin(std::make_shared<ImuFrameIdModifier>());

    // Shutdown the ROS 2 system
    rclcpp::shutdown();
    return 0;
}
