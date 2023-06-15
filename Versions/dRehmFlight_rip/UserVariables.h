#ifndef _USERVARIABLES_H_
#define _USERVARIABLES_H_ 

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

// SCALE FACTORS FOR PID //
extern float pScaleRoll;
extern float pScalePitch;
extern float pScaleYaw;
extern float iScaleRoll;
extern float iScalePitch;
extern float iScaleYaw;
extern float dScaleRoll;
extern float dScalePitch;
extern float dScaleYaw;

// PID GAINS FOR RIP //
extern const float Kp_alphaRoll;
extern const float Ki_alphaRoll;
extern const float Kd_alphaRoll;
extern const float Kp_betaPitch;
extern const float Ki_betaPitch;
extern const float Kd_betaPitch;

extern float pScaleAlpha;
extern float iScaleAlpha;
extern float dScaleAlpha;
extern float pScaleBeta;
extern float iScaleBeta;
extern float dScaleBeta;

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
