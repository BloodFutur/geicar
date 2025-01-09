#ifndef __steeringCmd_H
#define __steeringCmd_H

#include <cstdint>
#include <stdint.h>
#include "math.h"


#define STOP 50
#define MAX_PWM_LEFT 0
#define MAX_PWM_RIGHT 100

#define TOLERANCE_ANGLE 0.08


int steeringCmd(float requestedSteerAngle, float currentSteerAngle, uint8_t & steeringPwmCmd);
void newSteeringCmd(float front_wheel_rotation_, float currentFrontWheelRotation_, uint8_t & steeringPwmCmd);



#endif /*__ steeringCmd_H */