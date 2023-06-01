#include <Arduino.h>
#include <ArduinoEigenDense.h>


using namespace Eigen;


Vector3f pidOutput(Vector3f refState, Vector3f currState, Matrix3f P_gains, 
				Matrix3f I_gains, Matrix3f D_gains, float dt, bool lowThrottle) {
	static const float i_limit = 25.0f;
	static Vector3f integralOld(0,0,0);
	static Vector3f errorOld(0,0,0);
	Vector3f error;
	Vector3f integral(0,0,0);
	Vector3f derivative(0,0,0);
	Vector3f PIDValues(0,0,0);


  error = refState - currState;
	if (!lowThrottle) {
		integral = integralOld + error*dt;
	}
	for (int lv1=0; lv1<3; lv1++) {
		constrain(integral[lv1], -i_limit, i_limit);
	}
	derivative = (error - errorOld) / dt;

	integralOld = integral;
	errorOld = error;

  	PIDValues = 0.01f * (P_gains*error + I_gains*integral - D_gains*derivative);
	return PIDValues;

}
