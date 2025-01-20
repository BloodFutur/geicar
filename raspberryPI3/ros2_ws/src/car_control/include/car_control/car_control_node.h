#ifndef __car_control_node_H
#define __car_control_node_H

#include <stdint.h>
#include <string.h>  
#include "geometry_msgs/msg/twist.hpp"



#define PERIOD_UPDATE_CMD 100ms //Period to update proupulsion and steering command in [ms]

#define STOP 50 //PWM value to stop motors
#define KPI_LEFT 0.1109
#define KPI_RIGHT 0.1097

#define SPEED_ERR_THRESHOLD 1.0
#define WHEEL_BASE 0.55             //0.55m
#define WHEEL_RADIUS 0.095          //0.095m
#define ANGLE_MAX 0.401425728          //23°
#define MAX_PWM_MOTOR 100
#define MIN_PWM_MOTOR 10


#endif /*__ car_control_node_H */