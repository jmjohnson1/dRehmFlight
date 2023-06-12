#line 1 "/home/james/Documents/dRehmFlight/Versions/dRehmFlight_rip/pidController.h"
#ifndef _PIDCONTROLLER_H_
#define _PIDCONTROLLER_H_

/*
 * This is a translation of the original dRhemFlight ControlAngle() pid
 * controller. It was rewritten to use the Eigen linear algebra library.
*/
#include <Arduino.h>
#include <ArduinoEigen.h>
#include "GlobalVariables.h"


using namespace Eigen;

extern Vector3f desState;
extern Vector3f currState;
extern Vector3f pidOutputVals;
extern Vector2f desState_rip;
extern Vector2f currState_rip;
extern Vector2f pidOutputVals_rip;

Vector3f pidOutput(Vector3f refState, Vector3f currState, Matrix3f P_gains, 
									 Matrix3f I_gains, Matrix3f D_gains, bool lowThrottle);


Vector2f pidOutput_rip(Vector2f refState, Vector2f currState, Matrix2f P_gains, 
									 Matrix2f I_gains, Matrix2f D_gains, bool lowThrottle);

#endif
