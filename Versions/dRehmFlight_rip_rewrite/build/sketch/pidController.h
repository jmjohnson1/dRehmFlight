#line 1 "/home/james/Documents/dRehmFlight/Versions/dRehmFlight_rip_rewrite/pidController.h"
/*
 * This is a translation of the original dRhemFlight ControlAngle() pid
 * controller. It was rewritten to use the Eigen linear algebra library.
*/
#include <Arduino.h>
#include <ArduinoEigen.h>


using namespace Eigen;


Vector3f pidOutput(Vector3f refState, Vector3f currState, Matrix3f P_gains, 
									 Matrix3f I_gains, Matrix3f D_gains, float dt, bool lowThrottle,
									 float GyroX, float GyroY, float GyroZ) {

	//----- DECLARATIONS -----//
	static const float i_limit = 25.0f;
	// Declared as static so that the variables remain in memory while the program is running, even
	// outside of this function.
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
