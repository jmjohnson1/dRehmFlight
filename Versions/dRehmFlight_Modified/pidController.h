#include <Arduino.h>
#include <ArduinoEigenDense.h>


using namespace Eigen;


Vector3f pidOutput(Vector3f refState, Vector3f currState, Matrix3f P_gains, 
				Matrix3f I_gains, Matrix3f D_gains, float dt, bool lowThrottle,
				float GyroX, float GyroY, float GyroZ) {
	static const float i_limit = 25.0f;
	static Vector3f integralOld(0,0,0);
	static Vector3f errorOld(0,0,0);
	Vector3f error;
	Vector3f integral(0,0,0);
	Vector3f derivative(0,0,0);
	Vector3f PIDValues(0,0,0);


  error = refState - currState;
	error[1] = refState[1] - GyroZ;
	if (!lowThrottle) {
		integral = integralOld + error*dt;
	}
	for (int lv1=0; lv1<3; lv1++) {
		integral[lv1] = constrain(integral[lv1], -i_limit, i_limit);
	}
	derivative = (error - errorOld) / dt;
	derivative[0] = GyroY;
	derivative[2] = GyroX;

	integralOld = integral;
	errorOld = error;
	

  	PIDValues = 0.01f * (P_gains*error + I_gains*integral + D_gains*derivative);

	return PIDValues;

}
