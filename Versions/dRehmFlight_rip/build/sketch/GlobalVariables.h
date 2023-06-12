#line 1 "/home/james/Documents/dRehmFlight/Versions/dRehmFlight_rip/GlobalVariables.h"
#ifndef _GLOBALVARS_H_
#define _GLOBALVARS_H_

#include <Arduino.h>
#include <SD.h>
#include <ArduinoEigen.h>

using namespace Eigen;

extern float dt;
extern unsigned long current_time, prev_time;
extern unsigned long print_counter, serial_counter;
extern unsigned long blink_counter, blink_delay;
extern bool blinkAlternate;
extern unsigned long print_counterSD;

//Radio communication:
extern int channel_1_pwm, channel_2_pwm, channel_3_pwm, channel_4_pwm, channel_5_pwm, channel_6_pwm,
 channel_7_pwm, channel_8_pwm, channel_9_pwm, channel_10_pwm, channel_11_pwm, channel_12_pwm,
 channel_13_pwm;
extern int channel_1_pwm_prev, channel_2_pwm_prev, channel_3_pwm_prev, channel_4_pwm_prev;
extern int channel_1_pwm_pre, channel_2_pwm_pre, channel_3_pwm_pre, channel_4_pwm_pre;


//IMU:
extern float AccX, AccY, AccZ;
extern float AccX_prev, AccY_prev, AccZ_prev;
extern float GyroX, GyroY, GyroZ;
extern float GyroX_prev, GyroY_prev, GyroZ_prev;
extern float MagX, MagY, MagZ;
extern float MagX_prev, MagY_prev, MagZ_prev;
extern float roll_IMU, pitch_IMU, yaw_IMU;
extern float roll_IMU_prev, pitch_IMU_prev;
extern float q0;
extern float q1;
extern float q2;
extern float q3;

//Normalized desired state:
extern float thro_des, roll_des, pitch_des, yaw_des;
extern float roll_passthru, pitch_passthru, yaw_passthru;
extern float alphaRoll_des, betaPitch_des;

//Controller:
extern float roll_PID;
extern float pitch_PID; 
extern float yaw_PID;

//Mixer
extern float m1_command_scaled, m2_command_scaled, m3_command_scaled, m4_command_scaled, m5_command_scaled,
 m6_command_scaled;
extern int m1_command_PWM, m2_command_PWM, m3_command_PWM, m4_command_PWM, m5_command_PWM, m6_command_PWM;
extern float s1_command_scaled, s2_command_scaled, s3_command_scaled, s4_command_scaled, s5_command_scaled,
 s6_command_scaled, s7_command_scaled;
extern int s1_command_PWM, s2_command_PWM, s3_command_PWM, s4_command_PWM, s5_command_PWM, s6_command_PWM,
 s7_command_PWM;

// Flag for whether or not Iris is open
extern bool irisFlag;

// Joystick values
extern int alphaCounts;
extern int betaCounts;
extern float alpha;  
extern float beta;  
extern float alphaRoll;
extern float betaPitch;
extern float alphaOffset;
extern float betaOffset;

// Values for the setDesStateSerial() function
extern float serialInputValue;
extern float sineFrequency;
extern float sineTime;

// A flag for whether or not the sine sweep should be conducted
extern bool sweepFlag;     

// SD Card settings
extern String filePrefix;
extern String fileExtension;
extern String fileName;

extern File dataFile;

// PID vectors
extern Vector3f desState;
extern Vector3f currState;
extern Vector3f pidOutputVals;

// RIP PID vectors
extern Vector2f desState_rip;
extern Vector2f currState_rip;
extern Vector2f pidOutputVals_rip;

extern bool SD_is_present;

extern int cutoff_val;
extern int maxCutCounter;
extern int d_ch1;
extern int d_ch2;
extern int d_ch3;
extern int d_ch4;

extern int ch1_CutCounter;
extern int ch2_CutCounter;
extern int ch3_CutCounter;
extern int ch4_CutCounter;

extern bool doneWithSetup;
extern int servoLoopCounter;

// Number of loops before a sustained large change in values are accepted
extern int radioChCutoffTimeout;
extern bool failureFlag;

#endif
