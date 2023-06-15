#ifndef _PIDCONTROLLER_H_
#define _PIDCONTROLLER_H_

/*
 * This is a translation of the original dRhemFlight ControlAngle() pid
 * controller. It was rewritten to use the Eigen linear algebra library.
*/
#include <Arduino.h>
#include "GlobalVariables.h"
#include "UserVariables.h"


extern float errorOld_yaw;
extern float errorOld_alpha;
extern float errorOld_beta;
extern float integralOld_roll;
extern float integralOld_pitch;
extern float integralOld_yaw;
extern float integralOld_alpha;
extern float integralOld_beta;

void initializePID();
void anglePID();
void ripPID();


#endif
