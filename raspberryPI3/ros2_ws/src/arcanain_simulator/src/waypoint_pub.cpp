#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

using std::placeholders::_1;

class WaypointNavigation : public rclcpp::Node
{
public:
  WaypointNavigation()
  : Node("waypoint_pub"), current_waypoint_index(0)
  {
    waypoint_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("waypoint", 10);
    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, std::bind(&WaypointNavigation::odom_callback, this, _1));

    // Define waypoints
    waypoints.push_back(create_waypoint(6.0, 0.0, 0.0));
    waypoints.push_back(create_waypoint(0.0, 6.0, 0.0));
    waypoints.push_back(create_waypoint(-6.0, 0.0, 0.0));
    waypoints.push_back(create_waypoint(0.0, -6.0, 0.0));
    waypoints.push_back(create_waypoint(0.0, 0.0, 0.0));

    // Publish the first waypoint
    waypoint_pub->publish(waypoints[current_waypoint_index]);
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double goal_x = waypoints[current_waypoint_index].pose.position.x;
    double goal_y = waypoints[current_waypoint_index].pose.position.y;

    double distance_to_goal = std::sqrt(std::pow(x - goal_x, 2) + std::pow(y - goal_y, 2));

    if (distance_to_goal < 0.1) { // Check if the robot is close to the current goal
      current_waypoint_index = (current_waypoint_index + 1) % waypoints.size();
      waypoint_pub->publish(waypoints[current_waypoint_index]);
    } else {
      waypoint_pub->publish(waypoints[current_waypoint_index]);
    }
  }

  geometry_msgs::msg::PoseStamped create_waypoint(double x, double y, double theta)
  {
    geometry_msgs::msg::PoseStamped waypoint;
    waypoint.header.frame_id = "map";
    waypoint.pose.position.x = x;
    waypoint.pose.position.y = y;
    waypoint.pose.orientation.w = std::cos(theta / 2);
    waypoint.pose.orientation.z = std::sin(theta / 2);
    return waypoint;
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_pub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  std::vector<geometry_msgs::msg::PoseStamped> waypoints;
  int current_waypoint_index;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointNavigation>());
  rclcpp::shutdown();
  return 0;
}
