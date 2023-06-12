#include "Arduino.h"
#include "pidController.h"

using namespace Eigen;

const float i_limit = 25.0f;

// PID vectors
Vector3f desState(0,0,0);
Vector3f currState(0,0,0);
Vector3f pidOutputVals(0,0,0);

// RIP PID vectors
Vector2f desState_rip(0,0);
Vector2f currState_rip(0,0);
Vector2f pidOutputVals_rip(0,0);

Vector3f pidOutput(Vector3f refState, Vector3f currState, Matrix3f P_gains, 
									 Matrix3f I_gains, Matrix3f D_gains, bool lowThrottle) {

	//----- DECLARATIONS -----//
	static Vector3f integralOld(0,0,0);
	static Vector3f errorOld(0,0,0);
	Vector3f error;
	Vector3f integral(0,0,0);
	Vector3f derivative(0,0,0);
	Vector3f PIDValues(0,0,0);
	//----- END DECLARATIONS -----//
	
  error    = refState - currState;
	error[1] = refState[1] - GyroZ;

	// Prevent integral buildup with low throttle
	if (!lowThrottle) {
		integral = integralOld + error*dt;
	}

	// Saturate integral to prevent buildup
	for (int lv1=0; lv1<3; lv1++) {
		integral[lv1] = constrain(integral[lv1], -i_limit, i_limit);
	}

	derivative    = (error - errorOld) / dt;
	derivative[0] = GyroY;
	derivative[2] = GyroX;
	integralOld   = integral;
	errorOld      = error;
	PIDValues     = 0.01f * (P_gains*error + I_gains*integral + D_gains*derivative);

	return PIDValues;
}

Vector2f pidOutput_rip(Vector2f refState, Vector2f currState, Matrix2f P_gains, 
									 Matrix2f I_gains, Matrix2f D_gains, bool lowThrottle) {

	//----- DECLARATIONS -----//
	// Declared as static so that the variables remain in memory while the program is running, even
	// outside of this function.
	static Vector2f integralOld(0,0);
	static Vector2f errorOld(0,0);
	Vector2f error;
	Vector2f integral(0,0);
	Vector2f derivative(0,0);
	Vector2f PIDValues(0,0);
	//----- END DECLARATIONS -----//
	
	error = refState - currState;

	// Prevent integral buildup with low throttle
	if (!lowThrottle) {
		integral = integralOld + error*dt;
	}

	// Saturate integral to prevent buildup
	for (int lv1=0; lv1<2; lv1++) {
		integral[lv1] = constrain(integral[lv1], -i_limit, i_limit);
	}

	derivative    = (error - errorOld) / dt;
	integralOld   = integral;
	errorOld      = error;
	PIDValues     = (P_gains*error + I_gains*integral + D_gains*derivative);
	return PIDValues;
}
