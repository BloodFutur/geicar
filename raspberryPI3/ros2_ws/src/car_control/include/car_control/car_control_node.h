#ifndef __car_control_node_H
#define __car_control_node_H

#include <stdint.h>
#include <string.h>  


#define PERIOD_UPDATE_CMD 1ms //Period to update proupulsion and steering command in [ms]

#define STOP 50 //PWM value to stop motors
#define KP_LEFT 0.1109
#define KI_LEFT 0.0
#define KP_RIGHT 0.1097
#define KI_RIGHT 0.0


#endif /*__ car_control_node_H */