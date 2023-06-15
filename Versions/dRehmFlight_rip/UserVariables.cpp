#include "Arduino.h"
#include "UserVariables.h"

//================================================================================================//
//                                     USER-SPECIFIED VARIABLES                                   //                           
//================================================================================================//

//Radio failsafe values for every channel in the event that bad reciever data is detected. 
//Recommended defaults:
unsigned long channel_1_fs  = 1000; //thro cut
unsigned long channel_2_fs  = 1500; //ail neutral
unsigned long channel_3_fs  = 1500; //elev neutral
unsigned long channel_4_fs  = 1500; //rudd neutral
unsigned long channel_5_fs  = 2000; //gear, greater than 1500 = throttle cut
unsigned long channel_6_fs  = 1000; // Iris toggle (closed)
unsigned long channel_7_fs  = 1000; // Conduct sine sweep (Don't do a sine sweep)
unsigned long channel_8_fs  = 1000; // Perform step in pitch or roll (No step commands)
unsigned long channel_9_fs  = 1500; // Step angle (+15, 0, -15) (0 degrees)
unsigned long channel_10_fs = 1500; // P gain scale (no scaling)
unsigned long channel_11_fs = 1500; // I gain scale
unsigned long channel_12_fs = 1500; // D gain scale
unsigned long channel_13_fs = 1500; // Pitch and roll pid offset

//Filter parameters - Defaults tuned for 2kHz loop rate; 
//Do not touch unless you know what you are doing:
float B_madgwick = 0.04;  //Madgwick filter parameter
float B_accel = 0.14;     //Accelerometer LP filter paramter, (MPU6050 default: 0.14. MPU9250 default: 0.2)
float B_gyro = 0.1;       //Gyro LP filter paramter, (MPU6050 default: 0.1. MPU9250 default: 0.17)
float B_mag = 1.0;        //Magnetometer LP filter parameter

//Magnetometer calibration parameters - if using MPU9250, uncomment calibrateMagnetometer() in 
//void setup() to get these values, else just ignore these
float MagErrorX = 0.0;
float MagErrorY = 0.0; 
float MagErrorZ = 0.0;
float MagScaleX = 1.0;
float MagScaleY = 1.0;
float MagScaleZ = 1.0;

//IMU calibration parameters - calibrate IMU using calculate_IMU_error() in the void setup() to get
//these values, then comment out calculate_IMU_error()
float AccErrorX = 0.19;
float AccErrorY = 0.00;
float AccErrorZ = -0.06;
float GyroErrorX = -3.12;
float GyroErrorY = -0.52;
float GyroErrorZ = -0.08;
//Controller parameters (take note of defaults before modifying!): 
//Integrator saturation level, mostly for safety (default 25.0)
float i_limit = 25.0;
//Max roll/pitch angles in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode 
float maxRoll = 30.0;
float maxPitch = 30.0;
//Max yaw rate in deg/sec
float maxYaw = 160.0;

// MAXIMUM PENULUM ANGLES (INERTIAL) //
float maxAlphaRoll = 5.0f;
float maxBetaPitch = 5.0f;

// ANGLE MODE PID GAINS //
float Kp_roll_angle  = 0.15;
float Ki_roll_angle  = 0.03;
float Kd_roll_angle  = 0.05;
float Kp_pitch_angle = 0.15;
float Ki_pitch_angle = 0.03;
float Kd_pitch_angle = 0.05;

// YAW PID GAINS //
float Kp_yaw = 0.3;           
float Ki_yaw = 0.05;          
float Kd_yaw = 0.00015;       

// SCALE FACTORS FOR PID //
float pScaleRoll  = 1.0f;
float pScalePitch = 1.0f;
float pScaleYaw   = 1.0f;
float iScaleRoll  = 1.0f;
float iScalePitch = 1.0f;
float iScaleYaw   = 1.0f;
float dScaleRoll  = 1.0f;
float dScalePitch = 1.0f;
float dScaleYaw   = 1.0f;

// PID GAINS FOR RIP //
const float Kp_alphaRoll = -0.1f;
const float Ki_alphaRoll = -0.1f;
const float Kd_alphaRoll = -0.1f;
const float Kp_betaPitch  = -0.1f;
const float Ki_betaPitch  = -0.1f;
const float Kd_betaPitch  = -0.1f;

float pScaleAlpha = 1.0f;
float iScaleAlpha = 1.0f;
float dScaleAlpha = 1.0f;
float pScaleBeta  = 1.0f;
float iScaleBeta  = 1.0f;
float dScaleBeta  = 1.0f;

// JOYSTICK ANALOG INPUT RANGES //
int alphaCounts_min = 144;
int alphaCounts_max = 1023;
int betaCounts_min = 58;
int betaCounts_max = 1023;

// MAX AND MIN JOYSTICK ANGLES //
float alpha_min = -30;
float alpha_max = 30;
float beta_min = -30;
float beta_max = 30;

// Options for controlling the quad using user input values written over the serial line
// Sets whether or not to allow direct input of pitch or roll angles during a test flight. Setting
// this to true
bool useSerialAngleCommands = 0;  
// The axis to rotate about in the setDesStateSerial() function: 1 = roll, 2 = pitch
int axisToRotate = 1; 
// Determines whether or not to use a sine wave in the setDesStateSerial() function. If so, then the
// input from the serial line is taken to be the frequency of this sine wave in Hz.
bool useSineWave = 1; 

// Options for sine sweep
// Option for whether or not to do a sine sweep as part of a test flight. Setting this to true will
// result in pilot control of the axis connected to axisToRotate being removed, and a sine sweep
// conducted between the maximum and minimum frequencies specified.
bool conductSineSweep = 0; 	
float maxFreq = 1.5f; 			// Maximum frequency of the sine sweep in Hz
float minFreq = 0.05f; 			// Minimum frequency of the sine sweep in Hz
float sweepTime = 60;  	// How long to run the sweep for in seconds
