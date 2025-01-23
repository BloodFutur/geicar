#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <functional>
#include <memory>

#include "interfaces/msg/motors_order.hpp"
#include "interfaces/msg/motors_feedback.hpp"
#include "interfaces/msg/steering_calibration.hpp"
#include "interfaces/msg/joystick_order.hpp"

#include "std_srvs/srv/empty.hpp"
#include "std_msgs/msg/bool.hpp"


#include "../include/car_control/steeringCmd.h"
#include "../include/car_control/propulsionCmd.h"
#include "../include/car_control/car_control_node.h"


using namespace std;
using placeholders::_1;


class car_control : public rclcpp::Node {

public:
    car_control()
    : Node("car_control_node")
    {
        start = false;
        mode = 0;
        requestedThrottle = 0;
        requestedSteerAngle = 0;
        leftSpeedCmd = 0.0;
        rightSpeedCmd = 0.0;
        oldIntegratorLeftVal = 0.0;
        oldIntegratorRightVal = 0.0;
        targetRightSpeed = 0.0;
        targetLeftSpeed = 0.0;
        currentLeftSpeed = 0.0;
        currentRightSpeed = 0.0;
        
        obstacle_detected = false ;  

        pwm_cmd_pure_pursuit = 0;
        steering_angle_pwm_pure_pursuit = 0;
        angular_speed = 0;      

        publisher_can_= this->create_publisher<interfaces::msg::MotorsOrder>("motors_order", 10);

        publisher_steeringCalibration_ = this->create_publisher<interfaces::msg::SteeringCalibration>("steering_calibration", 10);

        

        subscription_joystick_order_ = this->create_subscription<interfaces::msg::JoystickOrder>(
        "joystick_order", 10, std::bind(&car_control::joystickOrderCallback, this, _1));

        subscription_motors_feedback_ = this->create_subscription<interfaces::msg::MotorsFeedback>(
        "motors_feedback", 10, std::bind(&car_control::motorsFeedbackCallback, this, _1));

        subscription_steering_calibration_ = this->create_subscription<interfaces::msg::SteeringCalibration>(
        "steering_calibration", 10, std::bind(&car_control::steeringCalibrationCallback, this, _1));

        subscription_obstacles_detection_ = this->create_subscription<std_msgs::msg::Bool>(
        "obstacles_detection", 10, std::bind(&car_control::obstaclesDetectionCallback, this, _1));
        
        server_calibration_ = this->create_service<std_srvs::srv::Empty>(
        "steering_calibration", std::bind(&car_control::steeringCalibration, this, std::placeholders::_1, std::placeholders::_2));

        subscription_output_pure_pursuit_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&car_control::outputPurePursuitCallback, this, _1));

        timer_ = this->create_wall_timer(PERIOD_UPDATE_CMD, std::bind(&car_control::updateCmd, this));

        
        RCLCPP_INFO(this->get_logger(), "car_control_node READY");
    }

    
private:

    /* Update start, mode, requestedThrottle, requestedSteerAngle and reverse from joystick order [callback function]  :
    *
    * This function is called when a message is published on the "/joystick_order" topic
    * 
    */
    void joystickOrderCallback(const interfaces::msg::JoystickOrder & joyOrder) {

        if (joyOrder.start != start){
            start = joyOrder.start;

            if (start)
                RCLCPP_INFO(this->get_logger(), "START");
            else 
                RCLCPP_INFO(this->get_logger(), "STOP");
        }
        

        if (joyOrder.mode != mode && joyOrder.mode != -1){ //if mode change
            mode = joyOrder.mode;

            if (mode==0){
                RCLCPP_INFO(this->get_logger(), "Switching to MANUAL Mode");
            }else if (mode==1){
                resetSpeedRegulationVariables();
                RCLCPP_INFO(this->get_logger(), "Switching to AUTONOMOUS Mode");
            }else if (mode==2){
                RCLCPP_INFO(this->get_logger(), "Switching to STEERING CALIBRATION Mode");
                startSteeringCalibration();
            }
        }
        
        if (mode == 0 && start){  //if manual mode -> update requestedThrottle, requestedSteerAngle and reverse from joystick order
            requestedThrottle = joyOrder.throttle;
            requestedSteerAngle = joyOrder.steer;
            reverse = joyOrder.reverse;
        }
    }

    /* Update currentAngle from motors feedback [callback function]  :
    *
    * This function is called when a message is published on the "/motors_feedback" topic
    * 
    */
    void motorsFeedbackCallback(const interfaces::msg::MotorsFeedback & motorsFeedback){
        currentAngle = motorsFeedback.steering_angle;
        currentLeftSpeed = motorsFeedback.left_rear_speed;
        currentRightSpeed = motorsFeedback.right_rear_speed;
    }

    //true if there is an obstacle and false if there is not
    void obstaclesDetectionCallback(const std_msgs::msg::Bool & detection){
        this->obstacle_detected = detection.data ;
    }


    //Callback function for the pure pursuit output
    void outputPurePursuitCallback(const geometry_msgs::msg::Twist & cmd_vel){
        double v = cmd_vel.linear.x;   
        double omega = -cmd_vel.angular.z;

        if (abs(omega) <= 0.3) {this->angular_speed = omega;}
        else {omega = this->angular_speed;}
        
        double steering_angle = 50.0;
        if (abs(v) > 0.0001) {
            steering_angle = std::atan(WHEEL_BASE * omega / v) / ANGLE_MAX;
            steering_angle = steering_angle*50 + 50;
        }
        steering_angle = std::clamp(steering_angle, 0.0, 100.0);
        this->steering_angle_pwm_pure_pursuit = static_cast<int>(steering_angle); 
        this->pwm_cmd_pure_pursuit = velocity_to_pwm(v);
    } 

    /* Update PWM commands : leftRearPwmCmd, rightRearPwmCmd, steeringPwmCmd
    *
    * This function is called periodically by the timer [see PERIOD_UPDATE_CMD in "car_control_node.h"]
    * 
    * In MANUAL mode, the commands depends on :
    * - requestedThrottle, reverse, requestedSteerAngle [from joystick orders]
    * - currentAngle [from motors feedback]
    */
    void updateCmd(){

        auto motorsOrder = interfaces::msg::MotorsOrder();

        if (!start){    //Car stopped
            leftRearPwmCmd = STOP;
            rightRearPwmCmd = STOP;
            steeringPwmCmd = STOP;


        }else{ //Car started

            //Manual Mode
            if (mode==0){
                
                manualPropulsionCmd(requestedThrottle, reverse, leftRearPwmCmd,rightRearPwmCmd);

                steeringCmd(requestedSteerAngle,currentAngle, steeringPwmCmd);


            //Autonomous Mode
            } else if (mode==1){
                if (!this->obstacle_detected){
                    // updateSpeedCmd();
                    
                    // RCLCPP_DEBUG(this->get_logger(), "leftSpeedCmd : %f", leftSpeedCmd);
                    // RCLCPP_DEBUG(this->get_logger(), "rightSpeedCmd : %f", rightSpeedCmd);

                    // autonomousPropulsionCmd(rightSpeedCmd, rightRearPwmCmd);
                    // autonomousPropulsionCmd(leftSpeedCmd, leftRearPwmCmd);
                    leftRearPwmCmd = this->pwm_cmd_pure_pursuit;
                    rightRearPwmCmd = this->pwm_cmd_pure_pursuit;
                    steeringPwmCmd = this->steering_angle_pwm_pure_pursuit;

                }
                else{
                    leftRearPwmCmd = STOP;
                    rightRearPwmCmd = STOP;
                    steeringPwmCmd = STOP;
                } 
            }
        }

        //Send order to motors
        motorsOrder.left_rear_pwm = leftRearPwmCmd;
        motorsOrder.right_rear_pwm = rightRearPwmCmd; 
        motorsOrder.steering_pwm = steeringPwmCmd;
        publisher_can_->publish(motorsOrder);
    }


    /* Start the steering calibration process :
    *
    * Publish a calibration request on the "/steering_calibration" topic
    */
    void startSteeringCalibration(){

        auto calibrationMsg = interfaces::msg::SteeringCalibration();
        calibrationMsg.request = true;

        RCLCPP_INFO(this->get_logger(), "Sending calibration request .....");
        publisher_steeringCalibration_->publish(calibrationMsg);
    }


    /* Function called by "steering_calibration" service
    * 1. Switch to calibration mode
    * 2. Call startSteeringCalibration function
    */
    void steeringCalibration([[maybe_unused]] std_srvs::srv::Empty::Request::SharedPtr req,
                            [[maybe_unused]] std_srvs::srv::Empty::Response::SharedPtr res)
    {

        mode = 2;    //Switch to calibration mode
        RCLCPP_WARN(this->get_logger(), "Switching to STEERING CALIBRATION Mode");
        startSteeringCalibration();
    }
    

    /* Manage steering calibration process [callback function]  :
    *
    * This function is called when a message is published on the "/steering_calibration" topic
    */
    void steeringCalibrationCallback (const interfaces::msg::SteeringCalibration & calibrationMsg){

        if (calibrationMsg.in_progress == true && calibrationMsg.user_need == false){
        RCLCPP_INFO(this->get_logger(), "Steering Calibration in progress, please wait ....");

        } else if (calibrationMsg.in_progress == true && calibrationMsg.user_need == true){
            RCLCPP_WARN(this->get_logger(), "Please use the buttons (L/R) to center the steering wheels.\nThen, press the blue button on the NucleoF103 to continue");
        
        } else if (calibrationMsg.status == 1){
            RCLCPP_INFO(this->get_logger(), "Steering calibration [SUCCESS]");
            RCLCPP_INFO(this->get_logger(), "Switching to MANUAL Mode");
            mode = 0;    //Switch to manual mode
            start = false;  //Stop car
        
        } else if (calibrationMsg.status == -1){
            RCLCPP_ERROR(this->get_logger(), "Steering calibration [FAILED]");
            RCLCPP_INFO(this->get_logger(), "Switching to MANUAL Mode");
            mode = 0;    //Switch to manual mode
            start = false;  //Stop car
        }
    
    }

    /* Reset speed regulation variables :
    *
    * This function reset all variables used for speed regulation 
    * when switching from MANUAL to AUTONOMOUS mode
    * 
    */
    void resetSpeedRegulationVariables(){
        leftSpeedCmd = 0.0;
        rightSpeedCmd = 0.0;
        oldIntegratorLeftVal = 0.0;
        oldIntegratorRightVal = 0.0;
        currentLeftSpeed = 0.0;
        currentRightSpeed = 0.0;
        targetRightSpeed = 0.0;
        targetLeftSpeed = 0.0;
    }

    /*Set car's speed
    *
    * This function set the speed of the car based on the speed command
    */
    void setCarSpeed(float LeftSpeed, float RightSpeed){
        targetLeftSpeed = LeftSpeed;
        targetRightSpeed = RightSpeed;
    }    

    /* Update speed commands for autonomous mode :
    *  This function is called by updateCmd function in AUTONOMOUS mode
    *  and update the speed command by calculating the integral and proportional part of the error
    */

   void updateSpeedCmd(){

        float errorLeftSpeed = 0.0;
        float errorRightSpeed = 0.0;
        float integratorLeftVal = 0.0;
        float integratorRightVal = 0.0;

        errorLeftSpeed = targetLeftSpeed - currentLeftSpeed;
        errorRightSpeed = targetRightSpeed - currentRightSpeed;

        if((errorLeftSpeed > SPEED_ERR_THRESHOLD) || (errorLeftSpeed < - SPEED_ERR_THRESHOLD)){
            integratorLeftVal = oldIntegratorLeftVal + KPI_LEFT * errorLeftSpeed;                    
            leftSpeedCmd = integratorLeftVal + KPI_LEFT * errorLeftSpeed;
            oldIntegratorLeftVal = integratorLeftVal;
        }

        if((errorRightSpeed > SPEED_ERR_THRESHOLD) || (errorRightSpeed < - SPEED_ERR_THRESHOLD)){
            integratorRightVal = oldIntegratorRightVal + KPI_RIGHT * errorRightSpeed;
            rightSpeedCmd = integratorRightVal + KPI_RIGHT * errorRightSpeed;
            oldIntegratorRightVal = integratorRightVal;
        }

        RCLCPP_DEBUG(this->get_logger(), "errorLeftSpeed : %f", errorLeftSpeed);
        RCLCPP_DEBUG(this->get_logger(), "errorRightSpeed : %f", errorRightSpeed);
   }


    // Function to convert velocity (m/s) to PWM value
    int velocity_to_pwm(double velocity) {
        // Convert velocity (m/s) to PWM
        int pwm = 50 + static_cast<int>(velocity * 60.0 / (2 * M_PI * WHEEL_RADIUS));
        pwm = std::clamp(pwm, MIN_PWM_MOTOR, MAX_PWM_MOTOR);
        return pwm;
    }

    
    // ---- Private variables ----

    //General variables
    bool start;
    int mode;    //0 : Manual    1 : Auto    2 : Calibration

    
    //Motors feedback variables
    float currentAngle;
    float currentLeftSpeed;
    float currentRightSpeed;

    //Manual Mode variables (with joystick control)
    bool reverse;
    float requestedThrottle;
    float requestedSteerAngle;

    //Autonomous Mode variables 
    float targetRightSpeed;
    float targetLeftSpeed;
    float leftSpeedCmd;
    float rightSpeedCmd;
    float oldIntegratorLeftVal;
    float oldIntegratorRightVal;
    float oldSpeedLeft;
    float oldSpeedRight;

    //Control variables
    uint8_t leftRearPwmCmd;
    uint8_t rightRearPwmCmd;
    uint8_t steeringPwmCmd;

    //Detection variables
    bool obstacle_detected;

    //Publishers
    rclcpp::Publisher<interfaces::msg::MotorsOrder>::SharedPtr publisher_can_;
    rclcpp::Publisher<interfaces::msg::SteeringCalibration>::SharedPtr publisher_steeringCalibration_;

    //Subscribers
    rclcpp::Subscription<interfaces::msg::JoystickOrder>::SharedPtr subscription_joystick_order_;
    rclcpp::Subscription<interfaces::msg::MotorsFeedback>::SharedPtr subscription_motors_feedback_;
    rclcpp::Subscription<interfaces::msg::SteeringCalibration>::SharedPtr subscription_steering_calibration_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_obstacles_detection_ ;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_output_pure_pursuit_;

    //Timer
    rclcpp::TimerBase::SharedPtr timer_;

    //Steering calibration Service
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr server_calibration_;

    //Pure pursuit command output
    int pwm_cmd_pure_pursuit;
    int steering_angle_pwm_pure_pursuit;
    float angular_speed;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<car_control>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}