#ifndef __propulsionCmd_H
#define __propulsionCmd_H

#include <cstdint>
#include <stdint.h>


#define MAX_SPEED 50.0
#define PWM_PLAGE 50.0
/* Calculate rightRearPwmCmd and leftRearPwmCmd (PWM) in MANUAL mode (from joystick orders)
*
* The joystick sends throttle order, which is directly transformed into PWM. The PWMs are equal for both motors in Manual Mode
* 
*/
int * manualPropulsionCmd(float requestedThrottle, bool reverse, uint8_t& leftRearPwmCmd, uint8_t& rightRearPwmCmd);
int * autonomousPropulsionCmd(float requestedSpeed, uint8_t& PwmCmd);

#endif /*__ propulsionCmd_H */