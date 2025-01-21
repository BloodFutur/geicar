#include "pure_pursuit_planner/pure_pursuit_planner_component.hpp"
#include "nav_msgs/msg/path.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

PurePursuitNode::PurePursuitNode()
: Node("pure_pursuit_planner") {
    // Parameter setting
    target_ind = 0;
    oldNearestPointIndex = -1;

    // Publisher
    cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    cmd_vel_rviz_pub = this->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel_rviz", 10);

    // Subscriber
    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
    "odometry/globalronan", 10, std::bind(&PurePursuitNode::odometry_callback, this, _1));
    path_sub = this->create_subscription<nav_msgs::msg::Path>(
            "path", 20,
            std::bind(&PurePursuitNode::path_callback, this, std::placeholders::_1));
    
    // Timer callback
    timer = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(dt * 1000)),
                                    std::bind(&PurePursuitNode::updateControl, this));
}

void PurePursuitNode::updateControl() {
    //Ajout 
    if (path_subscribe_flag) {
        auto [v, w] = purePursuitControl(target_ind);
        publishCmd(v, w);
    } else {
        RCLCPP_WARN(this->get_logger(), "Path data not available yet.");
    }
}

std::pair<double, double> PurePursuitNode::purePursuitControl(int& pind) {
    auto [ind, Lf] = searchTargetIndex();

    if (pind >= ind) {
        ind = pind;
    }

    double target_lookahed_x, target_lookahed_y, target_curvature;
    if (ind < static_cast<int>(cx.size())) {
        target_lookahed_x =cx[ind];
        target_lookahed_y = cy[ind];
        target_curvature = ck[ind];
    } else {
        target_lookahed_x = cx.back();
        target_lookahed_y = cy.back();
        target_curvature = ck.back();
        ind = static_cast<int>(cx.size()) - 1;
    }

    // target speed
    double curvature = std::max(minCurvature, std::min(abs(target_curvature), maxCurvature));
    curvature = curvature / maxCurvature;
    double target_vel = (maxVelocity - minVelocity) * pow(sin(acos(std::cbrt(curvature))), 3) + minVelocity; //[m/s]

    double alpha = std::atan2(target_lookahed_y - y, target_lookahed_x - x) - yaw;
    double v = target_vel;
    double w = v * std::tan(alpha) / Lf;

    pind = ind;

    RCLCPP_DEBUG(this->get_logger(), "Current position: x=%.2f, y=%.2f, yaw=%.2f", x, y, yaw);
    RCLCPP_DEBUG(this->get_logger(), "Target position: x=%.2f, y=%.2f", target_lookahed_x, target_lookahed_y);
    RCLCPP_DEBUG(this->get_logger(), "Target index: %d, Lf=%.2f", ind, Lf);
    RCLCPP_DEBUG(this->get_logger(), "Calculated: alpha=%.2f, v=%.2f, w=%.2f", alpha, v, w);

    return { v, w };
}

std::pair<int, double> PurePursuitNode::searchTargetIndex() {
    // Ajout 
    if (cx.empty() || cy.empty() || ck.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Path data is empty. Unable to calculate control.");
    }


    if (oldNearestPointIndex == -1) {
        std::vector<double> dx(cx.size()), dy(cy.size());
        for (size_t i = 0; i < cx.size(); ++i) {
            dx[i] = x - cx[i];
            dy[i] = y - cy[i];
        }
        std::vector<double> d(dx.size());
        std::transform(dx.begin(), dx.end(), dy.begin(), d.begin(), [](double dx, double dy) { return std::hypot(dx, dy); });
        auto it = std::min_element(d.begin(), d.end());
        oldNearestPointIndex = std::distance(d.begin(), it);
    } else {
        while (true) {
            double distanceThisIndex = calcDistance(cx[oldNearestPointIndex], cy[oldNearestPointIndex]);
            double distanceNextIndex = calcDistance(cx[oldNearestPointIndex + 1], cy[oldNearestPointIndex + 1]);
            if (distanceThisIndex < distanceNextIndex) {
                break;
            }
            oldNearestPointIndex++;
            if (oldNearestPointIndex >= static_cast<int>(cx.size()) - 1) {
                break;
            }
        }
    }

    double Lf = k * v + Lfc;

    int ind = oldNearestPointIndex;
    while (Lf > calcDistance(cx[ind], cy[ind])) {
        if (ind + 1 >= static_cast<int>(cx.size())) {
            break;
        }
        ind++;
    }

    return { ind, Lf };
}

double PurePursuitNode::calcDistance(double point_x, double point_y) const {
    double dx = x - point_x;
    double dy = y - point_y;
    return std::hypot(dx, dy);
}

void PurePursuitNode::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;

    tf2::Quaternion quat;
    tf2::fromMsg(msg->pose.pose.orientation, quat);
    tf2::Matrix3x3 mat(quat);
    double roll_tmp, pitch_tmp, yaw_tmp;
    mat.getRPY(roll_tmp, pitch_tmp, yaw_tmp);

    yaw = yaw_tmp;

    // Ajout
    // RCLCPP_INFO(this->get_logger(), "Odom received: x=%f, y=%f, yaw=%f", x, y, yaw);
}

void PurePursuitNode::path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
    if (!path_subscribe_flag) {
        for (const auto& pose : msg->poses) {
            cx.push_back(pose.pose.position.x);
            cy.push_back(pose.pose.position.y);
            ck.push_back(pose.pose.position.z);

            tf2::Quaternion quat;
            tf2::fromMsg(pose.pose.orientation, quat);
            tf2::Matrix3x3 mat(quat);
            double roll_rev, pitch_rev, yaw_rev;
            mat.getRPY(roll_rev, pitch_rev, yaw_rev);
            cyaw.push_back(yaw_rev);

            RCLCPP_INFO(this->get_logger(), "Received path point: (%f, %f)", pose.pose.position.x, pose.pose.position.y);
        }

        std::tie(target_ind, std::ignore) = searchTargetIndex();
    }

    path_subscribe_flag = true;
    // Ajout 
    // RCLCPP_INFO(this->get_logger(), "Path data received successfully.");
}

void PurePursuitNode::publishCmd(double v, double w)
{
    // Ajout 
    if (cx.empty() || cy.empty() || ck.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Path data is empty. Unable to calculate control.");
    }

    geometry_msgs::msg::Twist cmd_vel_msg;

    double goal_x = cx[cx.size() - 1];
    double goal_y = cy[cy.size() - 1];
    double goal_dist = std::abs(std::sqrt(std::pow((goal_x - x), 2.0) + std::pow((goal_y - y), 2.0)));

    // goal judgement
    if (goal_dist < goal_threshold) {
        std::cout << "Goal!" << std::endl;

        cmd_vel_msg.linear.x = 0.0;
        cmd_vel_msg.angular.z = 0.0;
    } else {
        cmd_vel_msg.linear.x = v;
        cmd_vel_msg.angular.z = w;
    }

    auto twist_stamped_msg = geometry_msgs::msg::TwistStamped();
    twist_stamped_msg.header.stamp = this->get_clock()->now();
    twist_stamped_msg.header.frame_id = "base_link";             
    twist_stamped_msg.twist = cmd_vel_msg;                     
    
    cmd_vel_rviz_pub->publish(twist_stamped_msg);
    cmd_vel_pub->publish(cmd_vel_msg);
}
