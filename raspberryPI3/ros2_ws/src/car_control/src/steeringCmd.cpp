#include "../include/car_control/steeringCmd.h"
#define FRONT_WHEEL_MAX_ROTATION 23.0f
#include <algorithm>

// Marcyle group function
void newSteeringCmd(float front_wheel_rotation_, float currentFrontWheelRotation_, uint8_t & steeringPwmCmd){

	float diff_front_wheel_rotation = (front_wheel_rotation_ - currentFrontWheelRotation_); 
	if (abs(diff_front_wheel_rotation / FRONT_WHEEL_MAX_ROTATION) < TOLERANCE_ANGLE){
		steeringPwmCmd = STOP;
	}
	else
	{
		steeringPwmCmd = 50 + std::clamp(static_cast<int>(50 * diff_front_wheel_rotation / FRONT_WHEEL_MAX_ROTATION), -50, 50);
	}
}
// Marcyle group function


//return the Pwm command to reach the angle passed in argument
int steeringCmd(float requestedSteerAngle, float currentSteerAngle, uint8_t & steeringPwmCmd){

	float errorAngle = currentSteerAngle - requestedSteerAngle;

    //Command's calculation
	if (abs(errorAngle)<TOLERANCE_ANGLE){
		steeringPwmCmd = STOP;
	}
	else {
		if (errorAngle>0) {
			steeringPwmCmd = MAX_PWM_LEFT;
		}
		else {
			steeringPwmCmd = MAX_PWM_RIGHT;
		}
	}

    return errorAngle;
}