#include "Arduino.h"
#include "pidController.h"
#include "GlobalVariables.h"
#include "UserVariables.h"
#include "filter.h"

// Keep track of last error term
float errorOld_yaw   = 0;
float errorOld_alpha = 0;
float errorOld_beta  = 0;

// Keep track of last integral term
float integralOld_roll  = 0;
float integralOld_pitch = 0;
float integralOld_yaw   = 0;
float integralOld_alpha = 0;
float integralOld_beta  = 0;

// Initialize dTerm filter
biquadFilter_s dTermFilter_roll;
biquadFilter_s dTermFilter_pitch;
biquadFilter_s dTermFilter_yaw;

void initializePID() {
	#ifndef PID_FILTERING
	#define PID_FILTERING
	biquadFilter_init(&dTermFilter_roll, 100.0f, 2000.0f);
	biquadFilter_init(&dTermFilter_pitch, 100.0f, 2000.0f);
	biquadFilter_init(&dTermFilter_yaw, 100.0f, 2000.0f);
	#endif
}


void anglePID() {

  // --- Roll --- //
  float error_roll = roll_des - roll_IMU;
  float integral_roll = integralOld_roll + error_roll*dt;
  if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
    integral_roll = 0;
  }
  //Saturate integrator to prevent unsafe buildup
  integral_roll = constrain(integral_roll, -i_limit, i_limit);
  float derivative_roll = GyroX;
	#ifdef PID_FILTERING
		derivative_roll = biquadFilter_apply(&dTermFilter_roll, derivative_roll);
	#endif
	//Scaled by .01 to bring within -1 to 1 range
  roll_PID = 0.01*(Kp_roll_angle*pScaleRoll*error_roll 
							   		+ Ki_roll_angle*iScaleRoll*integral_roll 
										- Kd_roll_angle*dScaleRoll*derivative_roll); 

  // --- Pitch --- //
  float error_pitch = pitch_des - pitch_IMU;
  float integral_pitch = integralOld_pitch + error_pitch*dt;
  if (channel_1_pwm < 1060) {
    integral_pitch = 0;
  }
  integral_pitch = constrain(integral_pitch, -i_limit, i_limit);
  float derivative_pitch = GyroY;
	#ifdef PID_FILTERING
		derivative_pitch = biquadFilter_apply(&dTermFilter_pitch, derivative_pitch);
	#endif
	pitch_PID = 0.01*(Kp_pitch_angle*pScalePitch*error_pitch 
										+ Ki_pitch_angle*iScalePitch*integral_pitch 
										- Kd_pitch_angle*dScalePitch*derivative_pitch);

  // --- Yaw --- // stablize on rate from GyroZ
  float error_yaw = yaw_des - GyroZ;
  float integral_yaw = integralOld_yaw + error_yaw*dt;
  if (channel_1_pwm < 1060) {
    integral_yaw = 0;
  }
  integral_yaw = constrain(integral_yaw, -i_limit, i_limit); 
  float derivative_yaw = (error_yaw - errorOld_yaw)/dt; 
	#ifdef PID_FILTERING
		derivative_yaw = biquadFilter_apply(&dTermFilter_yaw, derivative_yaw);
	#endif
  yaw_PID = 0.01*(Kp_yaw*pScaleYaw*error_yaw 
									+ Ki_yaw*iScaleYaw*integral_yaw 
									+ Kd_yaw*dScaleYaw*derivative_yaw);

  //Update roll variables
  integralOld_roll = integral_roll;
  //Update pitch variables
  integralOld_pitch = integral_pitch;
  //Update yaw variables
  errorOld_yaw = error_yaw;
  integralOld_yaw = integral_yaw;
}

void ripPID() {
  // --- Alpha --- //
  float error_alphaRoll = alphaRoll_des - alphaRoll;
  float integral_alphaRoll = integralOld_alpha + error_alphaRoll*dt;
  if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
    integral_alphaRoll = 0;
  }
  //Saturate integrator to prevent unsafe buildup
  integral_alphaRoll = constrain(integral_alphaRoll, -i_limit, i_limit);
  float derivative_alphaRoll = (error_alphaRoll - errorOld_alpha)/dt;

  roll_des = (Kp_alphaRoll*pScaleAlpha*error_alphaRoll 
							+ Ki_alphaRoll*iScaleAlpha*integral_alphaRoll 
							- Kd_alphaRoll*dScaleAlpha*derivative_alphaRoll); 

  // --- Beta --- //
  float error_betaPitch = betaPitch_des - betaPitch;
  float integral_betaPitch = integralOld_beta + error_betaPitch*dt;
  if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
    integral_betaPitch = 0;
  }
  //Saturate integrator to prevent unsafe buildup
  integral_betaPitch = constrain(integral_betaPitch, -i_limit, i_limit);
  float derivative_betaPitch = (error_betaPitch - errorOld_beta)/dt;

  pitch_des = (Kp_betaPitch*pScaleBeta*error_betaPitch 
							+ Ki_betaPitch*iScaleBeta*integral_betaPitch 
							- Kd_betaPitch*dScaleBeta*derivative_betaPitch); 
}
