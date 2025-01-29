#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

struct Obstacle
{
  double x, y;    // 障害物の中心座標 (メートル)
  double radius;  // 障害物の半径 (メートル)
  double margin;  // マージン (メートル)
};

class ObstaclePublisher : public rclcpp::Node
{
public:
  ObstaclePublisher()
  : Node("obstacle_pub")
  {
    global_obstacle_pub =
      this->create_publisher<visualization_msgs::msg::MarkerArray>("global_obstacle_markers", 10);
    local_obstacle_pub =
      this->create_publisher<visualization_msgs::msg::MarkerArray>("local_obstacle_markers", 10);
    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, std::bind(&ObstaclePublisher::odometry_callback, this, _1));
    timer_ = this->create_wall_timer(
      500ms, std::bind(&ObstaclePublisher::obstacle_publish, this));

    // 障害物のデータをobstaclesに追加
    obstacles.push_back({2.0, 0.0, 0.5, 0.2});    // 1つ目の障害物
    obstacles.push_back({2.0, 2.0, 0.5, 0.2});    // 2つ目の障害物
    obstacles.push_back({2.0, -2.0, 0.5, 0.2});   // 3つ目の障害物
    obstacles.push_back({0.0, 2.0, 0.5, 0.2});    // 4つ目の障害物
    obstacles.push_back({0.0, -2.0, 0.5, 0.2});   // 5つ目の障害物
    obstacles.push_back({-2.0, 0.0, 0.5, 0.2});   // 6つ目の障害物
    obstacles.push_back({-2.0, 2.0, 0.5, 0.2});   // 7つ目の障害物
    obstacles.push_back({-2.0, -2.0, 0.5, 0.2});  // 8つ目の障害物
    obstacles.push_back({4.0, 4.0, 0.5, 0.2});    // 9つ目の障害物
    obstacles.push_back({4.0, 1.0, 0.5, 0.2});    // 10つ目の障害物
    obstacles.push_back({4.0, -1.0, 0.5, 0.2});   // 11つ目の障害物
    obstacles.push_back({4.0, -4.0, 0.5, 0.2});   // 12つ目の障害物
    obstacles.push_back({-4.0, 4.0, 0.5, 0.2});   // 13つ目の障害物
    obstacles.push_back({-4.0, 1.0, 0.5, 0.2});   // 14つ目の障害物
    obstacles.push_back({-4.0, -1.0, 0.5, 0.2});  // 15つ目の障害物
    obstacles.push_back({-4.0, -4.0, 0.5, 0.2});  // 16つ目の障害物
    obstacles.push_back({1.0, 4.0, 0.5, 0.2});   // 17つ目の障害物
    obstacles.push_back({-1.0, 4.0, 0.5, 0.2});   // 18つ目の障害物
    obstacles.push_back({1.0, -4.0, 0.5, 0.2});  // 19つ目の障害物
    obstacles.push_back({-1.0, -4.0, 0.5, 0.2});  // 20つ目の障害物
  }

private:
  void obstacle_publish()
  {
    visualization_msgs::msg::MarkerArray marker_array;
    int id = 0;

    for (const auto & obstacle : obstacles) {
      // 障害物の中心位置と半径
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = rclcpp::Node::now();
      marker.ns = "global_obstacle_clusters";
      marker.id = id++;
      marker.type = visualization_msgs::msg::Marker::CYLINDER;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = obstacle.x;
      marker.pose.position.y = obstacle.y;
      marker.pose.position.z = 0.0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = obstacle.radius;
      marker.scale.y = obstacle.radius;
      marker.scale.z = 0.5;   // Height of the cylinder
      marker.color.a = 1.0;  // 透明度
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.5;

      marker_array.markers.push_back(marker);
    }

    global_obstacle_pub->publish(marker_array);

    // local obstacleの全てのマーカーをクリア
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.header.frame_id = "map";
    delete_marker.header.stamp = rclcpp::Node::now();
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    visualization_msgs::msg::MarkerArray delete_marker_array;
    delete_marker_array.markers.push_back(delete_marker);
    local_obstacle_pub->publish(delete_marker_array);
  }

  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // base_linkの位置を更新
    base_x = msg->pose.pose.position.x;
    base_y = msg->pose.pose.position.y;

    // // local obstacleの全てのマーカーをクリア
    // visualization_msgs::msg::Marker delete_marker;
    // delete_marker.header.frame_id = "map";
    // delete_marker.header.stamp = rclcpp::Node::now();
    // delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    // visualization_msgs::msg::MarkerArray delete_marker_array;
    // delete_marker_array.markers.push_back(delete_marker);
    // local_obstacle_pub->publish(delete_marker_array);

    visualization_msgs::msg::MarkerArray marker_array;
    int id = 0;

    for (const auto & obstacle : obstacles) {
      // base_linkからの距離を計算
      double distance = std::sqrt(
        std::pow(obstacle.x - base_x, 2) + std::pow(
          obstacle.y - base_y,
          2));

      // 半径2m以内にある障害物のみをパブリッシュ
      if (distance <= 2.0) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = rclcpp::Node::now();
        marker.ns = "local_obstacle_markers";
        marker.id = id++;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;
        // marker.pose.position.x = obstacle.x - base_x;  // base_link座標系での位置
        // marker.pose.position.y = obstacle.y - base_y;
        marker.pose.position.x = obstacle.x;  // map座標系での位置
        marker.pose.position.y = obstacle.y;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = obstacle.radius + 0.05;  // 直径を指定
        marker.scale.y = obstacle.radius + 0.05;
        marker.scale.z = 0.5 + 0.1;  // 高さは固定
        marker.color.a = 1.0;  // 透明度
        marker.color.r = 0.5;
        marker.color.g = 0.5;
        marker.color.b = 0.0;

        marker_array.markers.push_back(marker);
      }
    }

    // マーカー配列をパブリッシュ
    local_obstacle_pub->publish(marker_array);

    // マーカー配列をクリア
    //marker_array.markers.clear();
  }

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr global_obstacle_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr local_obstacle_pub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<Obstacle> obstacles;  // 障害物のリスト
  double base_x, base_y;  // base_linkの位置
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstaclePublisher>());
  rclcpp::shutdown();
  return 0;
}
