#include "pure_pursuit_planner/pure_pursuit_planner_component.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"


using std::placeholders::_1;
using namespace std::chrono_literals;

PurePursuitNode::PurePursuitNode()
: Node("pure_pursuit_planner") {
    // Parameter setting
    target_ind = 0;
    oldNearestPointIndex = -1;
    current_index_ = -1;
    autonomous_mode = false;

    // Publisher
    cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    cmd_vel_rviz_pub = this->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel_rviz", 10);
    gps_fix_pub = this->create_publisher<sensor_msgs::msg::NavSatFix>("gps/fix_adjusted", 10);

    // Subscriber
    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
    "odometry/globalronan", 10, std::bind(&PurePursuitNode::odometry_callback, this, _1));
    path_sub = this->create_subscription<nav_msgs::msg::Path>(
            "path", 20,
            std::bind(&PurePursuitNode::path_callback, this, std::placeholders::_1));
    autonomous_mode_sub = this->create_subscription<std_msgs::msg::Bool>(
            "autonomous_mode", 10,
            std::bind(&PurePursuitNode::autonomous_mode_callback, this, std::placeholders::_1));
    
    // Load CSV data
    if (!load_message("cmd_vel_demo_adjusted.csv")) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load CSV file");
        rclcpp::shutdown();
    }
    if (!load_gps_data("gps_fix_adjusted.csv")) {  // Charger le fichier GPS
        RCLCPP_ERROR(this->get_logger(), "Failed to load GPS CSV file");
        rclcpp::shutdown();
    }


    // Timer callback
    timer = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(dt * 1000)),
                                    std::bind(&PurePursuitNode::updateControl, this));

}

void PurePursuitNode::updateControl() {
    //Ajout 
    if (path_subscribe_flag && current_index_ != -1 && autonomous_mode) {
        // auto [v, w] = purePursuitControl(target_ind);
        // publishCmd(v, w);
        publish_demo_cmd();    
    }
}

bool PurePursuitNode::load_gps_data(const std::string &filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        return false;
    }
    std::string line;
    std::getline(file, line); // Skip header

    while (std::getline(file, line)) {
        std::vector<std::string> row;
        std::stringstream ss(line);
        std::string cell;

        while (std::getline(ss, cell, ',')) {
            row.push_back(cell);
        }

         // Convertir les éléments de row en double et ajouter à gps_data_
        std::vector<double> gps_data_row;
        try {
            gps_data_row.push_back(std::stod(row[0])); // Par exemple, temps GPS
            gps_data_row.push_back(std::stod(row[6])); // Latitude
            gps_data_row.push_back(std::stod(row[7])); // Longitude
            gps_data_row.push_back(std::stod(row[8])); // Altitude
            // Si tu as d'autres valeurs que tu veux ajouter, fais-le ici
        } catch (const std::invalid_argument &e) {
            RCLCPP_ERROR(this->get_logger(), "Erreur de conversion dans la ligne %d : %s", gps_index_, e.what());
            continue; // Passer à la ligne suivante en cas d'erreur de conversion
        }

        gps_data_.emplace_back(gps_data_row); // Ajouter la ligne convertie à gps_data_
    }

    gps_index_ = 0;
    return !gps_data_.empty();
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

    RCLCPP_INFO(this->get_logger(), "Current position: x=%.2f, y=%.2f, yaw=%.2f", x, y, yaw);
    RCLCPP_INFO(this->get_logger(), "Target position: x=%.2f, y=%.2f", target_lookahed_x, target_lookahed_y);
    RCLCPP_INFO(this->get_logger(), "Target index: %d, Lf=%.2f", ind, Lf);
    RCLCPP_INFO(this->get_logger(), "Calculated: alpha=%.2f, v=%.2f, w=%.2f", alpha, v, w);

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

void PurePursuitNode::autonomous_mode_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    autonomous_mode = msg->data;
    // RCLCPP_INFO(this->get_logger(), "Autonomous mode: %s", autonomous_mode ? "true" : "false");
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


bool PurePursuitNode::load_message(const std::string &filename)
{
    std::ifstream file(filename);
    if (!file.is_open()) {
        return false;
    }
    std::string line;
    std::getline(file, line);

    while (std::getline(file, line)) {
        std::vector<std::string> row;
        std::stringstream ss(line);
        std::string cell;

        while (std::getline(ss, cell, ',')) {
            row.push_back(cell);
        }
        csv_data_.emplace_back(row);
    }

    current_index_ = 0;
    return !csv_data_.empty();
}

void PurePursuitNode::publish_demo_cmd()
{
    if (current_index_ >= static_cast<int>(csv_data_.size())) {
        RCLCPP_INFO(this->get_logger(), "Finished publishing all rows");
        rclcpp::shutdown();
        return;
    }

    auto msg = geometry_msgs::msg::Twist();
    const auto &row = csv_data_[current_index_];

    // Parse CSV row into message
    try {
        msg.linear.x = std::stod(row[1]);
        msg.linear.y = std::stod(row[2]);
        msg.linear.z = std::stod(row[3]);
        msg.angular.x = std::stod(row[4]);
        msg.angular.y = std::stod(row[5]);
        msg.angular.z = std::stod(row[6]);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Error parsing row %d: %s", current_index_, e.what());
        rclcpp::shutdown();
        return;
    }

    // Publish message
    auto twist_stamped_msg = geometry_msgs::msg::TwistStamped();
    twist_stamped_msg.header.stamp = this->get_clock()->now();
    twist_stamped_msg.header.frame_id = "base_link";             
    twist_stamped_msg.twist = msg;                     
    
    cmd_vel_rviz_pub->publish(twist_stamped_msg);
    cmd_vel_pub->publish(msg);

    // Synchronize GPS and cmd_vel messages
    sync_gps_with_cmd_vel();

    // RCLCPP_INFO(this->get_logger(), "Published row %d", current_index_);

    current_index_++;
}

void PurePursuitNode::sync_gps_with_cmd_vel() {
    // Synchroniser les timestamps entre cmd_vel et gps
    if (gps_index_ >= static_cast<int>(gps_data_.size()) || current_index_ >= static_cast<int>(csv_data_.size())) {
        return;
    }

    const auto &gps_row = gps_data_[gps_index_];
    const auto &cmd_row = csv_data_[current_index_];

    double gps_time = gps_row[0];
    double cmd_time = std::stod(cmd_row[0]);

    if (gps_time < cmd_time) {
        sensor_msgs::msg::NavSatFix gps_msg;
        try {
            gps_msg.latitude = gps_row[1]; // Assurez-vous des indices ici (latitude)
            gps_msg.longitude = gps_row[2]; // Longitude
            gps_msg.altitude = gps_row[3]; // Altitude
            gps_msg.header.stamp = this->get_clock()->now();
            gps_msg.header.frame_id = "base_link";
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Error parsing GPS row %d: %s", gps_index_, e.what());
            return;
        }
        gps_fix_pub->publish(gps_msg);
        gps_index_++;
    }
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
