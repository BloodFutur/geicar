#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

class ImuFrameIdModifier : public rclcpp::Node {
public:
    ImuFrameIdModifier() : Node("imu_frame_id_modifier") {
        // Subscription to the input topic (/imu/data)
        subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10,
            std::bind(&ImuFrameIdModifier::imu_callback, this, std::placeholders::_1));

        // Publisher for the modified topic (/imu/modified_frame_id)
        publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/modified_frame_id", 10);
    }

private:
    // Callback function for the subscription
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        auto modified_msg = *msg;

        modified_msg.header.frame_id = "base_link"; 

        publisher_->publish(modified_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<ImuFrameIdModifier>());

    rclcpp::shutdown();
    return 0;
}
