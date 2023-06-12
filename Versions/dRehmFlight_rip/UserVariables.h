#ifndef _USERVARIABLES_H_
#define _USERVARIABLES_H_ 

#include <ArduinoEigen.h>

using namespace Eigen;

// DO NOT ASSIGN VARIABLES IN THIS FILE
//
//Radio failsafe values
extern unsigned long channel_1_fs;
extern unsigned long channel_2_fs;
extern unsigned long channel_3_fs;
extern unsigned long channel_4_fs;
extern unsigned long channel_5_fs;
extern unsigned long channel_6_fs;
extern unsigned long channel_7_fs;
extern unsigned long channel_8_fs;
extern unsigned long channel_9_fs;
extern unsigned long channel_10_fs;
extern unsigned long channel_11_fs;
extern unsigned long channel_12_fs;
extern unsigned long channel_13_fs;

//Filter parameters - Defaults tuned for 2kHz loop rate; 
extern float B_madgwick;
extern float B_accel;
extern float B_gyro;
extern float B_mag;

//Magnetometer calibration parameters
extern float MagErrorX;
extern float MagErrorY;
extern float MagErrorZ;
extern float MagScaleX;
extern float MagScaleY;
extern float MagScaleZ;

//IMU calibration parameters
extern float AccErrorX; 
extern float AccErrorY; 
extern float AccErrorZ; 
extern float GyroErrorX;
extern float GyroErrorY;
extern float GyroErrorZ;

//Controller parameters
extern float i_limit; 
extern float maxRoll; 
extern float maxPitch;
extern float maxYaw;  

// MAXIMUM PENULUM ANGLES (INERTIAL) //
extern float maxAlphaRoll;
extern float maxBetaPitch;

// ANGLE MODE PID GAINS //
extern float Kp_scale;
extern float Ki_scale;
extern float Kd_scale;

extern float Kp_roll_angle; 
extern float Ki_roll_angle; 
extern float Kd_roll_angle; 
extern float Kp_pitch_angle;
extern float Ki_pitch_angle;
extern float Kd_pitch_angle;

// YAW PID GAINS //
extern float Kp_yaw;
extern float Ki_yaw;
extern float Kd_yaw;

// MATRICES OF PID GIANS //
extern const Matrix3f P_gains;
extern const Matrix3f I_gains;
extern const Matrix3f D_gains;

// SCALE FACTORS FOR PID //
extern Matrix3f P_gainScale;
extern Matrix3f I_gainScale;
extern Matrix3f D_gainScale;

// PID GAINS FOR RIP //
extern const Matrix2f P_gains_rip;
extern const Matrix2f I_gains_rip;
extern const Matrix2f D_gains_rip;

extern Matrix2f P_gainScale_rip;
extern Matrix2f I_gainScale_rip;
extern Matrix2f D_gainScale_rip;

// JOYSTICK ANALOG INPUT RANGES //
extern int alphaCounts_min;
extern int alphaCounts_max;
extern int betaCounts_min;
extern int betaCounts_max;

// MAX AND MIN JOYSTICK ANGLES //
extern float alpha_min;
extern float alpha_max;
extern float beta_min;
extern float beta_max;

extern bool useSerialAngleCommands;
extern int axisToRotate;
extern bool useSineWave;

// Options for sine sweep
extern bool conductSineSweep;
extern float maxFreq;
extern float minFreq;
extern float sweepTime;

#endif
