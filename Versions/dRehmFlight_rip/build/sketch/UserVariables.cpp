#line 1 "/home/james/Documents/dRehmFlight/Versions/dRehmFlight_rip/UserVariables.cpp"
#include "Arduino.h"
#include "UserVariables.h"

using namespace Eigen;


//================================================================================================//
//                                     USER-SPECIFIED VARIABLES                                   //                           
//================================================================================================//

//Radio failsafe values for every channel in the event that bad reciever data is detected. 
//Recommended defaults:
unsigned long channel_1_fs = 1000; //thro cut
unsigned long channel_2_fs = 1500; //ail neutral
unsigned long channel_3_fs = 1500; //elev neutral
unsigned long channel_4_fs = 1500; //rudd neutral
unsigned long channel_5_fs = 2000; //gear, greater than 1500 = throttle cut
unsigned long channel_6_fs = 1000; // Iris toggle (closed)
unsigned long channel_7_fs = 1000; // Conduct sine sweep (Don't do a sine sweep)
unsigned long channel_8_fs = 1000; // Perform step in pitch or roll (No step commands)
unsigned long channel_9_fs = 1500; // Step angle (+15, 0, -15) (0 degrees)
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
float AccErrorX = 0.07;
float AccErrorY = 0.02;
float AccErrorZ = -0.02;
float GyroErrorX = -3.13;
float GyroErrorY = -0.76;
float GyroErrorZ = 0.36;

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
float Kp_scale = 0.75f;
float Ki_scale = 0.1f;
float Kd_scale = 1.1f;

float Kp_roll_angle = 0.2*Kp_scale*0.95;
float Ki_roll_angle = 0.3*Ki_scale*1.01;
float Kd_roll_angle = 0.05*Kd_scale*1.01;
float Kp_pitch_angle = 0.2*Kp_scale;
float Ki_pitch_angle = 0.3*Ki_scale;
float Kd_pitch_angle = 0.05*Kd_scale;

// YAW PID GAINS //
float Kp_yaw = 0.3;           
float Ki_yaw = 0.05;          
float Kd_yaw = 0.00015;       

// MATRICES OF PID GIANS //
const Matrix3f P_gains {{Kp_pitch_angle, 0, 0},
												{0, Kp_yaw, 0},
												{0, 0, Kp_roll_angle}};
const Matrix3f I_gains {{Ki_pitch_angle, 0, 0},
												{0, Ki_yaw, 0},
												{0, 0, Ki_roll_angle}};
const Matrix3f D_gains {{-Kd_pitch_angle, 0, 0},
												{0, Kd_yaw, 0},
												{0, 0, -Kd_roll_angle}};

// SCALE FACTORS FOR PID //
Matrix3f P_gainScale {{1, 0, 0},
											{0, 1, 0},
											{0, 0, 1}};
Matrix3f I_gainScale {{1, 0, 0},
											{0, 1, 0},
											{0, 0, 1}};
Matrix3f D_gainScale {{1, 0, 0},
											{0, 1, 0},
											{0, 0, 1}};

// PID GAINS FOR RIP //
const Matrix2f P_gains_rip {{-0.1f, 0},
														{0, -0.1f}};
const Matrix2f I_gains_rip {{0, 0},
														{0, 0}};
const Matrix2f D_gains_rip {{0, 0},
														{0, 0}};

Matrix2f P_gainScale_rip {{1, 0},
			 							      {0, 1}};
Matrix2f I_gainScale_rip {{1, 0},
			 									  {0, 1}};
Matrix2f D_gainScale_rip {{1, 0},
											    {0, 1}};

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
