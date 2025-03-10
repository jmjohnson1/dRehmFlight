// ArduinokTeensy Flight Controller - dRehmFlight
// Author: Nicholas Rehm
// Project Start: 1/6/2020
// Last Updated: 7/29/2022

//================================================================================================//

// CREDITS + SPECIAL THANKS
/*
Some elements inspired by:
http://www.brokking.net/ymfc-32_main.html

Madgwick filter function adapted from:
https://github.com/arduino-libraries/MadgwickAHRS

MPU9250 implementation based on MPU9250 library by:
brian.taylor@bolderflight.com
http://www.bolderflight.com

Thank you to:
RcGroups 'jihlein' - IMU implementation overhaul + SBUS implementation.
Everyone that sends me pictures and videos of your flying creations! -Nick

*/

//================================================================================================//
//                                    USER-SPECIFIED DEFINES
//                                    //
//================================================================================================//

// Uncomment only one receiver type
// #define USE_PWM_RX
// #define USE_PPM_RX
#define USE_SBUS_RX
// #define USE_DSM_RX
static const uint8_t num_DSM_channels = 6; // If using DSM RX, change this to match the number of
// transmitter channels you have

// Uncomment only one IMU
#define USE_MPU6050_I2C // Default
// #define USE_MPU9250_SPI

// Uncomment only one full scale gyro range (deg/sec)
#define GYRO_250DPS // Default
// #define GYRO_500DPS
// #define GYRO_1000DPS
// #define GYRO_2000DPS

// Uncomment only one full scale accelerometer range (G's)
#define ACCEL_2G // Default
                 // #define ACCEL_4G
                 // #define ACCEL_8G
                 // #define ACCEL_16G

// Define whether tuning RIP gains or core PID gains
#define TUNE_RIP
// #define TUNE_CORE

// Use OneShot125 or PWM
#define USE_ONESHOT

// Indicate if using rigid inverted pendulum (RIP)
//#define USE_RIP

// Defines an extreme rip angle to be at 15 degrees or greater
#define EXTREME_RIP_ANGLE 14.0f

// Use a biquad filter on the RIP D term
#define FILTER_D
//================================================================================================//

// REQUIRED LIBRARIES (included with download in main sketch folder)
#include "RingBuf.h" // Ring buffer used to store values for SD card
#include "SdFat.h"   // Library used for SD card
#include "TeensyTimerTool.h"
#include "filter.h" // For data filtering
#include "telemetry.h"
#include <PWMServo.h> //Commanding any extra actuators, installed with teensyduino installer
#include <SPI.h>      //SPI communication
#include <Wire.h>     //I2c communication
#include <string>

#if defined USE_SBUS_RX
#include "src/SBUS/SBUS.h" //sBus interface
#endif

#if defined USE_DSM_RX
#include "src/DSMRX/DSMRX.h"
#endif

#if defined USE_MPU6050_I2C
#include "src/MPU6050/MPU6050.h"
MPU6050 quadIMU;
MPU6050 ripIMU(MPU6050_ADDRESS_AD0_HIGH);
#elif defined USE_MPU9250_SPI
#include "src/MPU9250/MPU9250.h"
MPU9250 mpu9250(SPI2, 36);
#else
#error No MPU defined...
#endif

// SD card setup
// Logic needed to restart teensy
#define CPU_RESTART_ADDR (uint32_t *)0xE000ED0C
#define CPU_RESTART_VAL 0x5FA0004
#define CPU_RESTART (*CPU_RESTART_ADDR = CPU_RESTART_VAL);

// Use Teensy SDIO
#define SD_CONFIG SdioConfig(FIFO_SDIO)

// Interval between points (usec) for 100 samples/sec
#define LOG_INTERVAL_USEC 10000

// Size to log 256 byte lines at 100 Hz for more than ten minutes.
#define LOG_FILE_SIZE 256 * 100 * 600 // 150,000,000 bytes.

// Space to hold more than 1 second of 256-byte lines at 100 Hz in the buffer
#define RING_BUF_CAPACITY 50 * 512
#define LOG_FILENAME "SdioLogger.csv"

SdFs sd;
FsFile file;

// Ring buffer for filetype FsFile (The filemanager that will handle the data stream)
RingBuf<FsFile, RING_BUF_CAPACITY> buffer;

//================================================================================================//

// Setup gyro and accel full scale value selection and scale factor

#if defined USE_MPU6050_I2C
#define GYRO_FS_SEL_250 MPU6050_GYRO_FS_250
#define GYRO_FS_SEL_500 MPU6050_GYRO_FS_500
#define GYRO_FS_SEL_1000 MPU6050_GYRO_FS_1000
#define GYRO_FS_SEL_2000 MPU6050_GYRO_FS_2000
#define ACCEL_FS_SEL_2 MPU6050_ACCEL_FS_2
#define ACCEL_FS_SEL_4 MPU6050_ACCEL_FS_4
#define ACCEL_FS_SEL_8 MPU6050_ACCEL_FS_8
#define ACCEL_FS_SEL_16 MPU6050_ACCEL_FS_16
#elif defined USE_MPU9250_SPI
#define GYRO_FS_SEL_250 mpu9250.GYRO_RANGE_250DPS
#define GYRO_FS_SEL_500 mpu9250.GYRO_RANGE_500DPS
#define GYRO_FS_SEL_1000 mpu9250.GYRO_RANGE_1000DPS
#define GYRO_FS_SEL_2000 mpu9250.GYRO_RANGE_2000DPS
#define ACCEL_FS_SEL_2 mpu9250.ACCEL_RANGE_2G
#define ACCEL_FS_SEL_4 mpu9250.ACCEL_RANGE_4G
#define ACCEL_FS_SEL_8 mpu9250.ACCEL_RANGE_8G
#define ACCEL_FS_SEL_16 mpu9250.ACCEL_RANGE_16G
#endif

#if defined GYRO_250DPS
#define GYRO_SCALE GYRO_FS_SEL_250
#define GYRO_SCALE_FACTOR 131.0
#elif defined GYRO_500DPS
#define GYRO_SCALE GYRO_FS_SEL_500
#define GYRO_SCALE_FACTOR 65.5
#elif defined GYRO_1000DPS
#define GYRO_SCALE GYRO_FS_SEL_1000
#define GYRO_SCALE_FACTOR 32.8
#elif defined GYRO_2000DPS
#define GYRO_SCALE GYRO_FS_SEL_2000
#define GYRO_SCALE_FACTOR 16.4
#endif

#if defined ACCEL_2G
#define ACCEL_SCALE ACCEL_FS_SEL_2
#define ACCEL_SCALE_FACTOR 16384.0
#elif defined ACCEL_4G
#define ACCEL_SCALE ACCEL_FS_SEL_4
#define ACCEL_SCALE_FACTOR 8192.0
#elif defined ACCEL_8G
#define ACCEL_SCALE ACCEL_FS_SEL_8
#define ACCEL_SCALE_FACTOR 4096.0
#elif defined ACCEL_16G
#define ACCEL_SCALE ACCEL_FS_SEL_16
#define ACCEL_SCALE_FACTOR 2048.0
#endif

typedef struct attitudeInfo {
  float AccX, AccY, AccZ;
  float AccX_prev, AccY_prev, AccZ_prev;
  float GyroX, GyroY, GyroZ;
  float GyroX_prev, GyroY_prev, GyroZ_prev;
  float MagX, MagY, MagZ;
  float MagX_prev, MagY_prev, MagZ_prev;
  float roll_IMU, pitch_IMU, yaw_IMU;
  float roll_IMU_prev, pitch_IMU_prev;
  float q0 = 1.0f;
  float q1 = 0.0f;
  float q2 = 0.0f;
  float q3 = 0.0f;
  float AccErrorX = 0.0f;
  float AccErrorY = 0.0f;
  float AccErrorZ = 0.0f;
  float GyroErrorX = 0.0f;
  float GyroErrorY = 0.0f;
  float GyroErrorZ = 0.0f;
  // Filter parameters
  float B_madgwick = 0.04; // Madgwick filter parameter
  float B_accel = 0.14;    // Accelerometer LP filter paramter, (MPU6050 default: 0.14. MPU9250 default: 0.2)
  float B_gyro = 0.1;      // Gyro LP filter paramter, (MPU6050 default: 0.1. MPU9250 default: 0.17)
  float B_mag = 1.0;       // Magnetometer LP filter parameter
} attInfo;

//================================================================================================//
//                                     USER-SPECIFIED VARIABLES //
//================================================================================================//

// Radio failsafe values for every channel in the event that bad reciever data
// is detected. Recommended defaults:
unsigned long channel_1_fs = 1000;  // thro cut
unsigned long channel_2_fs = 1500;  // ail neutral
unsigned long channel_3_fs = 1500;  // elev neutral
unsigned long channel_4_fs = 1500;  // rudd neutral
unsigned long channel_5_fs = 2000;  // gear, greater than 1500 = throttle cut
unsigned long channel_6_fs = 1000;  // Iris toggle (closed)
unsigned long channel_7_fs = 1000;  // Conduct sine sweep (Don't do a sine sweep)
unsigned long channel_8_fs = 1000;  // Perform step in pitch or roll (No step commands)
unsigned long channel_9_fs = 1500;  // Step angle (+15, 0, -15) (0 degrees)
unsigned long channel_10_fs = 1500; // P gain scale (no scaling)
unsigned long channel_11_fs = 1500; // I gain scale
unsigned long channel_12_fs = 1500; // D gain scale
unsigned long channel_13_fs = 1500; // Pitch and roll pid offset
unsigned long channel_14_fs = 1000; // Reset switch

// Magnetometer calibration parameters - if using MPU9250, uncomment
// calibrateMagnetometer() in void setup() to get these values, else just ignore
// these
float MagErrorX = 0.0;
float MagErrorY = 0.0;
float MagErrorZ = 0.0;
float MagScaleX = 1.0;
float MagScaleY = 1.0;
float MagScaleZ = 1.0;

// Controller parameters (take note of defaults before modifying!):
// Integrator saturation level, mostly for safety (default 25.0)
float i_limit = 25.0;
// Max roll/pitch angles in degrees for angle mode (maximum ~70 degrees),
// deg/sec for rate mode
float maxRoll = 30.0;
float maxPitch = 30.0;
// Max yaw rate in deg/sec
float maxYaw = 160.0;

// MAXIMUM PENULUM ANGLES (INERTIAL) //
float maxRipRoll = 10.0f;
float maxRipPitch = 10.0f;

// ANGLE MODE PID GAINS //
// SCALE FACTORS FOR PID //
float pScaleRoll = 1.0f;
float pScalePitch = 1.0f;
float pScaleYaw = 1.0f;
float iScaleRoll = 1.0f;
float iScalePitch = 1.0f;
float iScaleYaw = 1.0f;
float dScaleRoll = 1.0f;
float dScalePitch = 1.0f;
float dScaleYaw = 1.0f;

#if defined USE_RIP
// Inner loop gains for fixed RIP
float Kp_roll_angle = 0.8627;
float Ki_roll_angle = 0.24;
float Kd_roll_angle = 0.1321;
float Kp_pitch_angle = 0.8627;
float Ki_pitch_angle = 0.24;
float Kd_pitch_angle = 0.1321;

// Inner loop gains for free RIP
float Kp_roll_angleFree = 1.8032;
float Ki_roll_angleFree = 0.0404;
float Kd_roll_angleFree = 0.2698;
float Kp_pitch_angleFree = 1.8032;
float Ki_pitch_angleFree = 0.0404;
float Kd_pitch_angleFree = 0.2698;
#else
float Kp_roll_angle = 1.8032;
float Ki_roll_angle = 0.0404;
float Kd_roll_angle = 0.2698;
float Kp_pitch_angle = 1.8032;
float Ki_pitch_angle = 0.0404;
float Kd_pitch_angle = 0.2698;
#endif

// Roll damping term for controlANGLE2(), lower is more damping (must be between 0 to 1)
float B_loop_roll = 0.9;
// Pitch damping term for controlANGLE2(), lower is more damping (must be between 0 to 1)
float B_loop_pitch = 0.9;

// RATE MODE PID GAINS //
float Kp_roll_rate = 0.15;
float Ki_roll_rate = 0.2;
float Kd_roll_rate = 0.0002;
float Kp_pitch_rate = 0.15;
float Ki_pitch_rate = 0.2;
float Kd_pitch_rate = 0.0002;

// YAW PID GAINS //
float Kp_yaw = 0.3;
float Ki_yaw = 0.06;
float Kd_yaw = 0.00015;

// PID GAINS FOR RIP //
// Here lies the first set of working gains. Take in their glory.
const float Kp_ripRoll = -8.2347f;
const float Ki_ripRoll = -429.13f;
const float Kd_ripRoll = -2.8501f;
const float Kp_ripPitch = -8.2347f;
const float Ki_ripPitch = -429.13f;
const float Kd_ripPitch = -2.8501f;
///////////////////////

float pScaleRipRoll = 1.0f;
float iScaleRipRoll = 1.0f;
float dScaleRipRoll = 1.0f;
float pScaleRipPitch = 1.0f;
float iScaleRipPitch = 1.0f;
float dScaleRipPitch = 1.0f;

// JOYSTICK ANALOG INPUT RANGES //
int joyRollCounts_min = 104;
int joyRollCounts_max = 828;
int joyPitchCounts_min = 231;
int joyPitchCounts_max = 835;

// MAX AND MIN JOYSTICK ANGLES //
float joyRollAngle_min = -30;
float joyRollAngle_max = 30;
float joyPitchAngle_min = -30;
float joyPitchAngle_max = 30;

// Options for controlling the quad using user input values written over the
// serial line Sets whether or not to allow direct input of pitch or roll angles
// during a test flight. Setting this to true
bool useSerialAngleCommands = 0;
// The axis to rotate about in the setDesStateSerial() function: 1 = roll, 2 =
// pitch
int axisToRotate = 1;
// Determines whether or not to use a sine wave in the setDesStateSerial()
// function. If so, then the input from the serial line is taken to be the
// frequency of this sine wave in Hz.
bool useSineWave = 1;

// Options for sine sweep
// Option for whether or not to do a sine sweep as part of a test flight.
// Setting this to true will result in pilot control of the axis connected to
// axisToRotate being removed, and a sine sweep conducted between the maximum
// and minimum frequencies specified.
bool conductSineSweep = 0;
float maxFreq = 3.0f;  // Maximum frequency of the sine sweep in Hz 		FIXME: Goes slightly beyond
											 // 																											maxFreq
float minFreq = 0.5f;  // Minimum frequency of the sine sweep in Hz
float sweepTime = 120; // How long to run the sweep for in seconds

//================================================================================================//
//                                      DECLARE PINS //
//================================================================================================//

// NOTE: Pin 13 is reserved for onboard LED, pins 18 and 19 are reserved for the
// MPU6050 IMU for
//  			default setup
// Radio note:
//  			If using SBUS, connect to pin 21 (RX5), if using DSM,
//  connect to pin 15 (RX3)
const int ch1Pin = 15; // throttle
const int ch2Pin = 16; // ail
const int ch3Pin = 17; // ele
const int ch4Pin = 20; // rudd
const int ch5Pin = 21; // gear (throttle cut)
const int ch6Pin = 22; // aux1 (free aux channel)
const int PPM_Pin = 23;
// TODO: Make the switch between oneshot and PWM servo happen with a precompile definition
// OneShot125 ESC pin outputs:
const int m1Pin = 6;
const int m2Pin = 7;
const int m3Pin = 10;
const int m4Pin = 9;
const int m5Pin = 4;
const int m6Pin = 5;
// PWM servo or ESC outputs:
const int servo1Pin = 8;
const int servo2Pin = 8;
const int servo3Pin = 8;
const int servo4Pin = 8;
const int servo5Pin = 8;
const int servo6Pin = 11;
const int servo7Pin = 12;

// Joystick pins
const int joyRollPin = 41;
const int joyPitchPin = 40;

// Pin and object for iris servo:
const int irisPin = 24;
PWMServo iris;

// Create servo objects to control a servo or ESC with PWM
PWMServo servo1;
PWMServo servo2;
PWMServo servo3;
PWMServo servo4;
PWMServo servo5;
PWMServo servo6;
PWMServo servo7;

//================================================================================================//

// DECLARE GLOBAL VARIABLES

// General stuff
float dt;

unsigned long current_time, prev_time;
unsigned long print_counter, serial_counter;
unsigned long blink_counter, blink_delay;
bool blinkAlternate;
unsigned long print_counterSD = 200000;

// Radio communication:
int channel_1_pwm, channel_2_pwm, channel_3_pwm, channel_4_pwm, channel_5_pwm, channel_6_pwm, channel_7_pwm,
    channel_8_pwm, channel_9_pwm, channel_10_pwm, channel_11_pwm, channel_12_pwm, channel_13_pwm, channel_14_pwm;
int channel_1_pwm_prev, channel_2_pwm_prev, channel_3_pwm_prev, channel_4_pwm_prev;
int channel_1_pwm_pre, channel_2_pwm_pre, channel_3_pwm_pre, channel_4_pwm_pre;

#if defined USE_SBUS_RX
SBUS sbus(Serial5);
uint16_t sbusChannels[16];
bool sbusFailSafe;
bool sbusLostFrame;
#endif
#if defined USE_DSM_RX
DSM1024 DSM;
#endif

attInfo quadIMU_info;
attInfo ripIMU_info;

// Normalized desired state:
float thro_des, roll_des, pitch_des, yaw_des;
float roll_passthru, pitch_passthru, yaw_passthru;
float ripRoll_des, ripPitch_des;

// Controller:
float error_roll, error_roll_prev, roll_des_prev, integral_roll, integral_roll_il, integral_roll_ol, integral_roll_prev,
    integral_roll_prev_il, integral_roll_prev_ol, derivative_roll, roll_PID = 0;
float error_pitch, error_pitch_prev, pitch_des_prev, integral_pitch, integral_pitch_il, integral_pitch_ol,
    integral_pitch_prev, integral_pitch_prev_il, integral_pitch_prev_ol, derivative_pitch, pitch_PID = 0;
float error_yaw, error_yaw_prev, integral_yaw, integral_yaw_prev, derivative_yaw, yaw_PID = 0;

float integralOld_ripRoll = 0;
float integralOld_ripPitch = 0;
float error_ripRoll = 0;
float integral_ripRoll = 0;
float derivative_ripRoll = 0;
float error_ripPitch = 0;
float integral_ripPitch = 0;
float derivative_ripPitch = 0;

// Mixer
float m1_command_scaled, m2_command_scaled, m3_command_scaled, m4_command_scaled, m5_command_scaled, m6_command_scaled;
int m1_command_PWM, m2_command_PWM, m3_command_PWM, m4_command_PWM, m5_command_PWM, m6_command_PWM;

// Timers for each motor
TeensyTimerTool::OneShotTimer m1_timer(TeensyTimerTool::TCK);
TeensyTimerTool::OneShotTimer m2_timer(TeensyTimerTool::TCK);
TeensyTimerTool::OneShotTimer m3_timer(TeensyTimerTool::TCK);
TeensyTimerTool::OneShotTimer m4_timer(TeensyTimerTool::TCK);

// Flag for whether or not the motors are busy writing
bool m1_writing = false;
bool m2_writing = false;
bool m3_writing = false;
bool m4_writing = false;

float s1_command_scaled, s2_command_scaled, s3_command_scaled, s4_command_scaled, s5_command_scaled, s6_command_scaled,
    s7_command_scaled;
int s1_command_PWM, s2_command_PWM, s3_command_PWM, s4_command_PWM, s5_command_PWM, s6_command_PWM, s7_command_PWM;

// Flag for whether or not Iris is open
bool irisFlag = false;

// Joystick values
int joyRollCounts;  // Joystick x-axis rotation analog signal
int joyPitchCounts; // Joystick y-axis rotation analog signal
float joyRoll;      // Joystick x-axis rotation angle
float joyPitch;     // Joystick x-axis rotation angle
float ripRoll;
float ripPitch;

// Full range of analog input based on calibration
float fullRange_joyRoll = joyRollCounts_max - joyRollCounts_min;
float fullRange_joyPitch = joyPitchCounts_max - joyRollCounts_min;
float fullRange_joyRoll_half = fullRange_joyRoll / 2.0f;
float fullRange_joyPitch_half = fullRange_joyPitch / 2.0f;

float joyRollOffset = 0.0f;
float joyPitchOffset = 0.0f;

// Values for the setDesStateSerial() function
float serialInputValue = 0.0f; // User input over the serial line
float sineFrequency = 0.0f;    // If using sine wave, its frequency in Hz
float sineTime = 0.0f;         // Counter used to determine time in the sine functions (seconds)

// A flag for whether or not the sine sweep should be conducted. User input
// while the program is running sets this. DON'T SET THIS YOURSELF!
bool sweepFlag = 0;

// SD Card settings
String filePrefix = "flight_data";
String fileExtension = ".csv";
String fileName;

bool SD_is_present = 0;

int cutoff_val = 150;
int d_ch1;
int d_ch2;
int d_ch3;
int d_ch4;

int ch1_CutCounter = 0;
int ch2_CutCounter = 0;
int ch3_CutCounter = 0;
int ch4_CutCounter = 0;

bool doneWithSetup = 0;
int servoLoopCounter = 0;

// Number of loops before a sustained large change in values are accepted
int radioChCutoffTimeout = 10;
bool failureFlag = 0;

int throttleCutCount = 0;

// For keeping track of loop times
float max_loopTime = 0;

// Activates if the RIP angle is too large
bool extremeAngleFlag = 0;

// Flag to check if the flight loop has started yet, prevents lock in main loop when throttle killed
bool flightLoopStarted = 0;

int loopCount = 0;

// RIP D-gain filter
biquadFilter_t ripRollDFilter;
biquadFilter_t ripPitchDFilter;

// Telemetry
Telemetry telem;

//========================================================================================================================//
//                                                      VOID SETUP //
//========================================================================================================================//

void setup() {
  Serial.begin(500000); // USB serial
  delay(500);

  // Initialize all pins
  pinMode(13, OUTPUT); // Pin 13 LED blinker on board, do not modify

  pinMode(m1Pin, OUTPUT);
  pinMode(m2Pin, OUTPUT);
  pinMode(m3Pin, OUTPUT);
  pinMode(m4Pin, OUTPUT);
  pinMode(m5Pin, OUTPUT);
  pinMode(m6Pin, OUTPUT);

  // Initialize timers for OneShot125
  m1_timer.begin(m1_EndPulse);
  m2_timer.begin(m2_EndPulse);
  m3_timer.begin(m3_EndPulse);
  m4_timer.begin(m4_EndPulse);

  servo1.attach(servo1Pin, 1000, 2100); // Pin, min PWM value, max PWM value
  servo2.attach(servo2Pin, 1000, 2100);
  servo3.attach(servo3Pin, 1000, 2100);
  servo4.attach(servo4Pin, 1000, 2100);
  servo5.attach(servo5Pin, 1000, 2100);
  servo6.attach(servo6Pin, 1000, 2100);
  servo7.attach(servo7Pin, 1000, 2100);

  // Attach the iris servo and close it
  iris.attach(irisPin);

  // Tare the joystick angle
  closeIris();
  delay(2500);
  Serial.println("Iris closed");
  getJoyAngle();
  joyRollOffset = -joyRoll;
  joyPitchOffset = -joyPitch;

  // Set built in LED to turn on to signal startup
  digitalWrite(13, HIGH);

  delay(5);

  // Initialize radio communication (defined in header file)
  radioSetup();

  // Begin mavlink telemetry module
  telem.InitTelemetry();

  // Set radio channels to default (safe) values before entering main loop
  channel_1_pwm = channel_1_fs;
  channel_2_pwm = channel_2_fs;
  channel_3_pwm = channel_3_fs;
  channel_4_pwm = channel_4_fs;
  channel_5_pwm = channel_5_fs;
  channel_6_pwm = channel_6_fs;
  channel_7_pwm = channel_7_fs;
  channel_8_pwm = channel_8_fs;
  channel_9_pwm = channel_9_fs;
  channel_10_pwm = channel_10_fs;
  channel_11_pwm = channel_11_fs;
  channel_12_pwm = channel_12_fs;
  channel_13_pwm = channel_13_fs;
  channel_14_pwm = channel_14_fs;

  // Initialize IMU communication
#if defined USE_RIP
  IMUinit(&ripIMU);
#endif
  IMUinit(&quadIMU);

  biquadFilter_init(&ripRollDFilter, 1000, 2000);
  biquadFilter_init(&ripPitchDFilter, 1000, 2000);

  // Initialize the SD card, returns 1 if no sd card is detected or it can't be
  // initialized
  SD_is_present = !logData_setup();

  delay(2500);

  // Get IMU error to zero accelerometer and gyro readings, assuming vehicle is
  // level when powered up Calibration parameters printed to serial monitor.
  // Paste these in the user specified variables section, then comment this out
  // forever.
  // calculate_IMU_error(&ripIMU_info, &ripIMU);
  ripIMU_info.AccErrorX = 0.04;
  ripIMU_info.AccErrorY = 0.02;
  ripIMU_info.AccErrorZ = -0.02;
  ripIMU_info.GyroErrorX = -2.20;
  ripIMU_info.GyroErrorY = 0.55;
  ripIMU_info.GyroErrorZ = 0.06;
  // calculate_IMU_error(&quadIMU_info, &quadIMU);
  quadIMU_info.AccErrorX = -0.02;
  quadIMU_info.AccErrorY = 0.02;
  quadIMU_info.AccErrorZ = 0.01;
  quadIMU_info.GyroErrorX = 2.44;
  quadIMU_info.GyroErrorY = 1.16;
  quadIMU_info.GyroErrorZ = 0.85;

  // Arm servo channels
  servo1.write(0); // Command servo angle from 0-180 degrees (1000 to 2000 PWM)
  servo2.write(0); // Set these to 90 for servos if you do not want them to
                   // briefly max out on startup
  servo3.write(0); // Keep these at 0 if you are using servo outputs for motors
  servo4.write(0);
  servo5.write(0);
  servo6.write(0);
  servo7.write(0);

  delay(5);

  // PROPS OFF. Uncomment this to calibrate your ESCs by setting throttle stick
  // to max, powering on, and lowering throttle to zero after the beeps
  // calibrateESCs();
  // Code will not proceed past here if this function is uncommented!

  // Arm OneShot125 motors
  m1_command_PWM = 125; // Command OneShot125 ESC from 125 to 250us pulse length
  m2_command_PWM = 125;
  m3_command_PWM = 125;
  m4_command_PWM = 125;
  m5_command_PWM = 125;
  m6_command_PWM = 125;
  armMotors(); // Loop over commandMotors() until ESCs happily arm

  // Calibrate the joystick. Will be in an infinite loop.
  // calibrateJoystick();

  // Indicate entering main loop with 3 quick blinks
  setupBlink(3, 160, 70); // numBlinks, upTime (ms), downTime (ms)

  // If using MPU9250 IMU, uncomment for one-time magnetometer calibration (may
  // need to repeat for new locations) Generates magentometer error and scale
  // factors to be pasted in user-specified variables section
  // calibrateMagnetometer();

  doneWithSetup = 1;
}

//================================================================================================//
//                                      MAIN LOOP //
//================================================================================================//
void loop() {
  // Keep track of what time it is and how much time has elapsed since the last loop
  prev_time = current_time;
  current_time = micros();
  dt = (current_time - prev_time) / 1000000.0;

  loopBlink(); // Indicate we are in main loop with short blink every 1.5 seconds

  //  Print data at 100hz (uncomment one at a time for troubleshooting) - SELECT ONE:
  //  Prints radio pwm values (expected: 1000 to 2000)
  // printRadioData();
  //  Prints desired vehicle state commanded in either degrees or deg/sec (expected: +/- maxAXIS for roll, pitch, yaw; 0
  //  to 1 for throttle)
  // printDesiredState();
  //  Prints filtered gyro data direct from IMU (expected: ~ -250 to 250, 0 at rest)
  // printGyroData();
  //  Prints filtered accelerometer data direct from IMU (expected: ~ -2 to 2; x,y 0 when level, z 1 when level)
  // printAccelData();
  //  Prints filtered magnetometer data direct from IMU (expected: ~ -300 to 300)
  // printMagData();
  //  Prints roll, pitch, and yaw angles in degrees from Madgwick filter (expected: degrees, 0 when level)
  // printRollPitchYaw();
  //  Prints computed stabilized PID variables from controller and desired setpoint (expected: ~ -1 to 1)
  // printPIDoutput();
  //  Prints the values being written to the motors (expected: 120 to 250)
  // printMotorCommands();
  //  Prints the values being written to the servos (expected: 0 to 180)
  // printServoCommands();
  //  Prints the time between loops in microseconds (expected: microseconds between loop iterations)
  // printLoopRate();
  //  Prints the angles alpha, beta, pitch, roll, alpha + roll, beta + pitch
  // printRIPAngles();

  //  Prints desired and imu roll state for serial plotter
  // displayRoll();
  //  Prints desired and imu pitch state for serial plotter
  // displayPitch();

  // Check if rotors should be armed
  if (!flightLoopStarted && channel_5_pwm < 1500) {
    flightLoopStarted = 1;
    telem.SetSystemState(MAV_STATE_ACTIVE);
    telem.SetSystemMode(MAV_MODE_MANUAL_ARMED);
  }

  // Check for whether the iris should be open
  if ((channel_6_pwm < 1750) || extremeAngleFlag) {
    irisFlag = 0;
    closeIris();
  } else {
    irisFlag = 1;
    openIris();
  }

  ripExtremeAngleCheck();
  if (channel_6_pwm < 1750) {
    extremeAngleFlag = 0;
  }

  // Sine sweep check
  if (channel_7_pwm > 1750) {
    conductSineSweep = 1;
  } else {
    conductSineSweep = 0;
    sineTime = 0;
  }

  // Write to SD card buffer
  if (SD_is_present && (current_time - print_counterSD) > LOG_INTERVAL_USEC) {
    print_counterSD = micros();
    logData_writeBuffer();
  }

  if (loopCount > 2000) {
    telem.SendHeartbeat();
    loopCount = 0;
  }
  if ((loopCount % 250) == 0) {
		telem.UpdateReceived();
    // SendAttitude(quadIMU_info.roll_IMU, quadIMU_info.pitch_IMU, quadIMU_info.yaw_IMU,
    //							quadIMU_info.GyroX, quadIMU_info.GyroY, quadIMU_info.GyroZ);
    telem.SendAttitude(quadIMU_info.roll_IMU, quadIMU_info.pitch_IMU, 0.0f, quadIMU_info.GyroX, quadIMU_info.GyroY,
                       0.0f);
    telem.SendPIDGains_rip(Kp_ripRoll * pScaleRipRoll, Ki_ripRoll * iScaleRipRoll, Kd_ripRoll * dScaleRipRoll);
    telem.SendPIDGains_core(Kp_roll_angle * pScaleRoll, Ki_roll_angle * iScaleRoll, Kd_roll_angle * dScaleRoll);
  }
  loopCount++;

  // Get vehicle state
  getIMUData(&quadIMU_info,
             &quadIMU); // Pulls raw gyro, accelerometer, and magnetometer data from IMU and LP filters to remove noise
  Madgwick6DOF(&quadIMU_info, dt); // Updates roll_IMU, pitch_IMU, and yaw_IMU angle estimates (degrees)

  // Get RIP state
  getIMUData(&ripIMU_info, &ripIMU);
  Madgwick6DOF(&ripIMU_info, dt);

  // Get the joystick angle from potentiometers
  getJoyAngle();

  // Compute desired state based on radio inputs
  getDesState(); // Convert raw commands to normalized values based on saturated control limits

  if (useSerialAngleCommands) {
    // Overwrites axisToRotate in getDesState()
    setDesStateSerial(axisToRotate);
  }

  if (conductSineSweep) {
    // Overwrites axisToRotate in getDesState()
    performSineSweep(axisToRotate);
  }

  if (channel_8_pwm > 1250 && channel_8_pwm < 1750) {
    rollStep();
  }

  if (channel_8_pwm > 1750) {
    pitchStep();
  }

  getPScale();
  getIScale();
  getDScale();
  scaleBoth();

#if defined USE_RIP
  // RIP PID (Sets/overwrites roll_des and pitch_des)
  if (irisFlag) {
    ripPID();
  }
#endif

  // PID Controller - SELECT ONE:
  controlANGLE();
  // controlANGLE2(); //Stabilize on angle setpoint using cascaded method. Rate controller must be tuned well first!
  // controlRATE(); //Stabilize on rate setpoint

  // Actuator mixing and scaling to PWM values
  controlMixer();  // Mixes PID outputs to scaled actuator commands -- custom
                   // mixing assignments done here
  scaleCommands(); // Scales motor commands to 125 to 250 range (oneshot125
                   // protocol) and servo PWM commands to 0 to 180 (for servo
                   // library)

  // Throttle cut check
  bool killThrottle = throttleCut(); // Directly sets motor commands to low
                                     // based on state of ch5

  commandMotors(); // Sends command pulses to each motor pin using OneShot125 protocol
  // Command actuators
  servo1.write(s1_command_PWM); // Writes PWM value to servo object
  servo2.write(s2_command_PWM);
  servo3.write(s3_command_PWM);
  servo4.write(s4_command_PWM);

  // Get vehicle commands for next loop iteration
  getCommands(); // Pulls current available radio commands
  failSafe();    // Prevent failures in event of bad receiver connection, defaults
                 // to failsafe values assigned in setup

  if (killThrottle && (throttleCutCount > 100) && flightLoopStarted) {
    logData_endProcess();
    while (1) {
      getCommands();
      commandMotors();
      servo1.write(s1_command_PWM); // Writes PWM value to servo object
      servo2.write(s2_command_PWM);
      servo3.write(s3_command_PWM);
      servo4.write(s4_command_PWM);

      // Check for whether or not the iris should be open
      if (channel_6_pwm < 1500) {
        irisFlag = 0;
        closeIris();
      } else {
        irisFlag = 1;
        openIris();
      }

      if (channel_14_pwm > 1500) {
        CPU_RESTART;
      }
    }
  } else if (killThrottle && flightLoopStarted) {
    throttleCutCount++;
    Serial.print("throttleCutCount: ");
    Serial.println(throttleCutCount);
  } else {
    throttleCutCount = 0;
  }

  //unsigned long time = micros() - current_time;
  //if (time > max_loopTime) {
  //  max_loopTime = time;
  //}
  //Serial.print("Time = ");
  //Serial.print(time);
  //Serial.print(" Max = ");
  //Serial.println(max_loopTime);

  // Regulate loop rate
  loopRate(2000); // Do not exceed 2000Hz, all filter parameters tuned to 2000Hz by default
}

//========================================================================================================================//
//                                                      FUNCTIONS //
//========================================================================================================================//

void controlMixer() {
  // DESCRIPTION: Mixes scaled commands from PID controller to actuator outputs based on vehicle configuration
  /*
   * Takes roll_PID, pitch_PID, and yaw_PID computed from the PID controller and appropriately mixes
   * them for the desired vehicle configuration. For example on a quadcopter, the left two motors
   * should have +roll_PID while the right two motors should have -roll_PID. Front two should have
   * -pitch_PID and the back two should have +pitch_PID etc... every motor has normalized (0 to 1)
   * thro_des command for throttle control. Can also apply direct unstabilized commands from the
   * transmitter with
   * roll_passthru, pitch_passthru, and yaw_passthu. mX_command_scaled and sX_command scaled
   * variables are used in scaleCommands() in preparation to be sent to the motor ESCs and servos.
   *
   * Relevant variables:
   * thro_des - direct thottle control
   * roll_PID, pitch_PID, yaw_PID - stabilized axis variables
   * roll_passthru, pitch_passthru, yaw_passthru - direct unstabilized command passthrough
   * channel_6_pwm - free auxillary channel, can be used to toggle things with an 'if' statement
   */

  // 0.5 is centered servo, 0.0 is zero throttle if connecting to ESC for
  // conventional PWM, 1.0 is max throttle
  m1_command_scaled = thro_des - pitch_PID + roll_PID + yaw_PID; // Front Left
  m2_command_scaled = thro_des - pitch_PID - roll_PID - yaw_PID; // Front Right
  m3_command_scaled = thro_des + pitch_PID - roll_PID + yaw_PID; // Back Right
  m4_command_scaled = thro_des + pitch_PID + roll_PID - yaw_PID; // Back Left
  s1_command_scaled = thro_des - pitch_PID + roll_PID + yaw_PID; // Front Left
  s2_command_scaled = thro_des - pitch_PID - roll_PID - yaw_PID; // Front Right
  s3_command_scaled = thro_des + pitch_PID - roll_PID + yaw_PID; // Back Right
  s4_command_scaled = thro_des + pitch_PID + roll_PID - yaw_PID; // Back Left
  s5_command_scaled = 0;
  s6_command_scaled = 0;
  s7_command_scaled = 0;
}

void IMUinit(MPU6050 *imuObj) {
// DESCRIPTION: Initialize IMU
#if defined USE_MPU6050_I2C
  Wire.begin();
  Wire.setClock(1000000); // Note this is 2.5 times the spec sheet 400 kHz
                          // max...

  imuObj->initialize();

  if (imuObj->testConnection() == false) {
    Serial.println("MPU6050 initialization unsuccessful");
    Serial.println("Check MPU6050 wiring or try cycling power");
    while (1) {
    }
  }

  // From the reset state all registers should be 0x00, so we should be at
  // max sample rate with digital low pass filter(s) off.  All we need to
  // do is set the desired fullscale ranges
  imuObj->setFullScaleGyroRange(GYRO_SCALE);
  imuObj->setFullScaleAccelRange(ACCEL_SCALE);

#elif defined USE_MPU9250_SPI
  int status = mpu9250.begin();

  if (status < 0) {
    Serial.println("MPU9250 initialization unsuccessful");
    Serial.println("Check MPU9250 wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while (1) {
    }
  }

  // From the reset state all registers should be 0x00, so we should be at
  // max sample rate with digital low pass filter(s) off.  All we need to
  // do is set the desired fullscale ranges
  mpu9250.setGyroRange(GYRO_SCALE);
  mpu9250.setAccelRange(ACCEL_SCALE);
  mpu9250.setMagCalX(MagErrorX, MagScaleX);
  mpu9250.setMagCalY(MagErrorY, MagScaleY);
  mpu9250.setMagCalZ(MagErrorZ, MagScaleZ);
  mpu9250.setSrd(0); // sets gyro and accel read to 1khz, magnetometer read to 100hz
#endif
}

void getIMUData(attInfo *imu, MPU6050 *mpu6050) {
  int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
  mpu6050->getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);

  // Accelerometer
  imu->AccX = AcX / ACCEL_SCALE_FACTOR; // G's
  imu->AccY = AcY / ACCEL_SCALE_FACTOR;
  imu->AccZ = AcZ / ACCEL_SCALE_FACTOR;
  // Correct the outputs with the calculated error values
  imu->AccX = imu->AccX - imu->AccErrorX;
  imu->AccY = imu->AccY - imu->AccErrorY;
  imu->AccZ = imu->AccZ - imu->AccErrorZ;
  // LP filter accelerometer data
  imu->AccX = (1.0 - imu->B_accel) * imu->AccX_prev + imu->B_accel * imu->AccX;
  imu->AccY = (1.0 - imu->B_accel) * imu->AccY_prev + imu->B_accel * imu->AccY;
  imu->AccZ = (1.0 - imu->B_accel) * imu->AccZ_prev + imu->B_accel * imu->AccZ;
  imu->AccX_prev = imu->AccX;
  imu->AccY_prev = imu->AccY;
  imu->AccZ_prev = imu->AccZ;

  // Gyro
  imu->GyroX = GyX / GYRO_SCALE_FACTOR; // deg/sec
  imu->GyroY = GyY / GYRO_SCALE_FACTOR;
  imu->GyroZ = GyZ / GYRO_SCALE_FACTOR;
  // Correct the outputs with the calculated error values
  imu->GyroX = imu->GyroX - imu->GyroErrorX;
  imu->GyroY = imu->GyroY - imu->GyroErrorY;
  imu->GyroZ = imu->GyroZ - imu->GyroErrorZ;
  // LP filter gyro data
  imu->GyroX = (1.0 - imu->B_gyro) * imu->GyroX_prev + imu->B_gyro * imu->GyroX;
  imu->GyroY = (1.0 - imu->B_gyro) * imu->GyroY_prev + imu->B_gyro * imu->GyroY;
  imu->GyroZ = (1.0 - imu->B_gyro) * imu->GyroZ_prev + imu->B_gyro * imu->GyroZ;
  imu->GyroX_prev = imu->GyroX;
  imu->GyroY_prev = imu->GyroY;
  imu->GyroZ_prev = imu->GyroZ;
}

void calculate_IMU_error(attInfo *imu, MPU6050 *mpu6050) {
  int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
  imu->AccErrorX = 0.0;
  imu->AccErrorY = 0.0;
  imu->AccErrorZ = 0.0;
  imu->GyroErrorX = 0.0;
  imu->GyroErrorY = 0.0;
  imu->GyroErrorZ = 0.0;

  // Read IMU values 12000 times
  int c = 0;
  while (c < 12000) {
    mpu6050->getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);

    imu->AccX = AcX / ACCEL_SCALE_FACTOR;
    imu->AccY = AcY / ACCEL_SCALE_FACTOR;
    imu->AccZ = AcZ / ACCEL_SCALE_FACTOR;
    imu->GyroX = GyX / GYRO_SCALE_FACTOR;
    imu->GyroY = GyY / GYRO_SCALE_FACTOR;
    imu->GyroZ = GyZ / GYRO_SCALE_FACTOR;

    // Sum all readings
    imu->AccErrorX = imu->AccErrorX + imu->AccX;
    imu->AccErrorY = imu->AccErrorY + imu->AccY;
    imu->AccErrorZ = imu->AccErrorZ + imu->AccZ;
    imu->GyroErrorX = imu->GyroErrorX + imu->GyroX;
    imu->GyroErrorY = imu->GyroErrorY + imu->GyroY;
    imu->GyroErrorZ = imu->GyroErrorZ + imu->GyroZ;
    c++;
  }
  // Divide the sum by 12000 to get the error value
  imu->AccErrorX = imu->AccErrorX / c;
  imu->AccErrorY = imu->AccErrorY / c;
  imu->AccErrorZ = imu->AccErrorZ / c - 1.0;
  imu->GyroErrorX = imu->GyroErrorX / c;
  imu->GyroErrorY = imu->GyroErrorY / c;
  imu->GyroErrorZ = imu->GyroErrorZ / c;

  Serial.print("float AccErrorX = ");
  Serial.print(imu->AccErrorX);
  Serial.println(";");
  Serial.print("float AccErrorY = ");
  Serial.print(imu->AccErrorY);
  Serial.println(";");
  Serial.print("float AccErrorZ = ");
  Serial.print(imu->AccErrorZ);
  Serial.println(";");

  Serial.print("float GyroErrorX = ");
  Serial.print(imu->GyroErrorX);
  Serial.println(";");
  Serial.print("float GyroErrorY = ");
  Serial.print(imu->GyroErrorY);
  Serial.println(";");
  Serial.print("float GyroErrorZ = ");
  Serial.print(imu->GyroErrorZ);
  Serial.println(";");

  Serial.println("Paste these values in user specified variables section and "
                 "comment out calculate_IMU_error() in void setup.");
  for (;;)
    ;
}

void calibrateAttitude() {
  // DESCRIPTION: Used to warm up the main loop to allow the madwick filter to
  // converge before commands can be sent to the actuators Assuming vehicle is
  // powered up on level surface!
  /*
   * This function is used on startup to warm up the attitude estimation and is
   * what causes startup to take a few seconds to boot.
   */
  // Warm up IMU and madgwick filter in simulated main loop
  for (int i = 0; i <= 10000; i++) {
    prev_time = current_time;
    current_time = micros();
    dt = (current_time - prev_time) / 1000000.0;
    getIMUData(&quadIMU_info, &quadIMU);
    Madgwick6DOF(&quadIMU_info, dt);
    loopRate(2000); // do not exceed 2000Hz
  }
}

void Madgwick6DOF(attInfo *imu, float invSampleFreq) {
  // DESCRIPTION: Attitude estimation through sensor fusion - 6DOF
  /*
   * See description of Madgwick() for more information. This is a 6DOF
   * implimentation for when magnetometer data is not available (for example
   * when using the recommended MPU6050 IMU for the default setup).
   */
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  float gx = imu->GyroX;
  float gy = -imu->GyroY;
  float gz = -imu->GyroZ;
  float ax = -imu->AccX;
  float ay = imu->AccY;
  float az = imu->AccZ;

  // Convert gyroscope degrees/sec to radians/sec
  gx *= 0.0174533f;
  gy *= 0.0174533f;
  gz *= 0.0174533f;

  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-imu->q1 * gx - imu->q2 * gy - imu->q3 * gz);
  qDot2 = 0.5f * (imu->q0 * gx + imu->q2 * gz - imu->q3 * gy);
  qDot3 = 0.5f * (imu->q0 * gy - imu->q1 * gz + imu->q3 * gx);
  qDot4 = 0.5f * (imu->q0 * gz + imu->q1 * gy - imu->q2 * gx);

  // Compute feedback only if accelerometer measurement valid (avoids NaN in
  // accelerometer normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * imu->q0;
    _2q1 = 2.0f * imu->q1;
    _2q2 = 2.0f * imu->q2;
    _2q3 = 2.0f * imu->q3;
    _4q0 = 4.0f * imu->q0;
    _4q1 = 4.0f * imu->q1;
    _4q2 = 4.0f * imu->q2;
    _8q1 = 8.0f * imu->q1;
    _8q2 = 8.0f * imu->q2;
    q0q0 = imu->q0 * imu->q0;
    q1q1 = imu->q1 * imu->q1;
    q2q2 = imu->q2 * imu->q2;
    q3q3 = imu->q3 * imu->q3;

    // Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * imu->q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * imu->q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * imu->q3 - _2q1 * ax + 4.0f * q2q2 * imu->q3 - _2q2 * ay;
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= imu->B_madgwick * s0;
    qDot2 -= imu->B_madgwick * s1;
    qDot3 -= imu->B_madgwick * s2;
    qDot4 -= imu->B_madgwick * s3;
  }

  // Integrate rate of change of quaternion to yield quaternion
  imu->q0 += qDot1 * invSampleFreq;
  imu->q1 += qDot2 * invSampleFreq;
  imu->q2 += qDot3 * invSampleFreq;
  imu->q3 += qDot4 * invSampleFreq;

  // Normalise quaternion
  recipNorm = invSqrt(imu->q0 * imu->q0 + imu->q1 * imu->q1 + imu->q2 * imu->q2 + imu->q3 * imu->q3);
  imu->q0 *= recipNorm;
  imu->q1 *= recipNorm;
  imu->q2 *= recipNorm;
  imu->q3 *= recipNorm;

  // Compute angles
  imu->roll_IMU = atan2(imu->q0 * imu->q1 + imu->q2 * imu->q3, 0.5f - imu->q1 * imu->q1 - imu->q2 * imu->q2) *
                  57.29577951;                                                           // degrees
  imu->pitch_IMU = -asin(-2.0f * (imu->q1 * imu->q3 - imu->q0 * imu->q2)) * 57.29577951; // degrees
  imu->yaw_IMU = -atan2(imu->q1 * imu->q2 + imu->q0 * imu->q3, 0.5f - imu->q2 * imu->q2 - imu->q3 * imu->q3) *
                 57.29577951; // degrees
}

void setDesStateSerial(int controlledAxis) {
  // DESCRIPTION: Sets the desired pitch and roll angles based on user input
  // over USB
  if (Serial.available()) {
    serialInputValue = Serial.parseFloat();
    while (Serial.available() != 0) {
      Serial.read();
    }
  }

  float desiredAngle = 0;

  if (useSineWave) {
    sineFrequency = static_cast<float>(serialInputValue);
    desiredAngle = 10 * sin(2 * PI * sineFrequency * sineTime); // Set the output to be a sin wave
                                                                // between -5 and 5 degrees
    sineTime = sineTime + 1 / 2000.0f;
  } else {
    desiredAngle = static_cast<float>(serialInputValue);
  }

  switch (controlledAxis) {
  case 1:
    roll_des = desiredAngle;
    break;
  case 2:
    pitch_des = desiredAngle;
    break;
  default:
    break;
  }
}

void performSineSweep(int controlledAxis) {
  // DESCRIPTION: Performs a sine sweep from minFreq (Hz) to maxFreq (Hz) over sweepTime (seconds)
  float desiredAngle = 0;
  float amp = 10; // Sine wave amplitude in degrees

  // if (Serial.available()) {
  //   sweepFlag = 1;
  //   while (Serial.available() !=0) {
  //       Serial.read();
  //     }
  // }

  // if (sweepFlag){
  desiredAngle =
      amp * sin(PI * (maxFreq - minFreq) / pow(sweepTime, 2) * pow(sineTime, 3) + 2 * PI * minFreq * sineTime);
  if (sineTime > sweepTime) {
    desiredAngle = 0;
  }
  sineTime = sineTime + 1 / 2000.0f;
  //}

  switch (controlledAxis) {
  case 1:
    roll_des = desiredAngle;
    break;
  case 2:
    pitch_des = desiredAngle;
    break;
  default:
    break;
  }
}

void rollStep() {
  float desiredAngle;
  if (channel_9_pwm < 1250) {
    desiredAngle = 10.0f;
  } else if (channel_9_pwm > 1750) {
    desiredAngle = -10.0f;
  } else {
    desiredAngle = 0.0f;
  }
  // roll_des = desiredAngle;
}
void pitchStep() {
  float desiredAngle;
  if (channel_9_pwm < 1250) {
    desiredAngle = 3.0f;
  } else if (channel_9_pwm > 1750) {
    desiredAngle = -3.0f;
  } else {
    desiredAngle = 0.0f;
  }
  // pitch_des = desiredAngle;
  ripPitch_des = desiredAngle;
}

void getDesState() {
  // DESCRIPTION: Normalizes desired control values to appropriate values
  /*
   * Updates the desired state variables thro_des, roll_des, pitch_des, and
   * yaw_des. These are computed by using the raw RC pwm commands and scaling
   * them to be within our limits defined in setup. thro_des stays within 0 to 1
   * range. roll_des and pitch_des are scaled to be within max roll/pitch amount
   * in either degrees (angle mode) or degrees/sec (rate mode). yaw_des is
   * scaled to be within max yaw in degrees/sec. Also creates roll_passthru,
   * pitch_passthru, and yaw_passthru variables, to be used in commanding
   * motors/servos with direct unstabilized commands in controlMixer().
   */
  thro_des = (channel_1_pwm - 1000.0) / 1000.0; // Between 0 and 1
  roll_des = (channel_2_pwm - 1500.0) / 500.0;  // Between -1 and 1
  pitch_des = (channel_3_pwm - 1500.0) / 500.0; // Between -1 and 1
  yaw_des = (channel_4_pwm - 1500.0) / 500.0;   // Between -1 and 1
  roll_passthru = roll_des / 2.0;               // Between -0.5 and 0.5
  pitch_passthru = pitch_des / 2.0;             // Between -0.5 and 0.5
  yaw_passthru = yaw_des / 2.0;                 // Between -0.5 and 0.5

  ripRoll_des = roll_des;
  ripPitch_des = pitch_des;

  // Constrain within normalized bounds
  thro_des = constrain(thro_des, 0.0, 1.0);               // Between 0 and 1
  roll_des = constrain(roll_des, -1.0, 1.0) * maxRoll;    // Between -maxRoll and +maxRoll
  pitch_des = constrain(pitch_des, -1.0, 1.0) * maxPitch; // Between -maxPitch and +maxPitch
  yaw_des = constrain(yaw_des, -1.0, 1.0) * maxYaw;       // Between -maxYaw and +maxYaw
  roll_passthru = constrain(roll_passthru, -0.5, 0.5);
  pitch_passthru = constrain(pitch_passthru, -0.5, 0.5);
  yaw_passthru = constrain(yaw_passthru, -0.5, 0.5);

  ripRoll_des = constrain(ripRoll_des, -1.0, 1.0) * maxRipRoll;
  ripPitch_des = constrain(ripPitch_des, -1.0, 1.0) * maxRipPitch;
}

void ripPID() {
  // --- Rip Roll --- //
  error_ripRoll = ripRoll_des - ripIMU_info.roll_IMU;
  integral_ripRoll = integralOld_ripRoll + error_ripRoll * dt;
  if (channel_1_pwm < 1060) { // Don't let integrator build if throttle is too low
    integral_ripRoll = 0;
  }
  // Saturate integrator to prevent unsafe buildup
  integral_ripRoll = constrain(integral_ripRoll, -i_limit, i_limit);
  derivative_ripRoll = -ripIMU_info.GyroX;
#if defined FILTER_D
  derivative_ripRoll = biquadFilter_apply(&ripRollDFilter, derivative_ripRoll);
#endif

  roll_des = (Kp_ripRoll * pScaleRipRoll * error_ripRoll + Ki_ripRoll * iScaleRipRoll * integral_ripRoll +
              Kd_ripRoll * dScaleRipRoll * derivative_ripRoll);

  // --- Beta --- //
  error_ripPitch = ripPitch_des - ripIMU_info.pitch_IMU;
  integral_ripPitch = integralOld_ripPitch + error_ripPitch * dt;
  if (channel_1_pwm < 1060) { // Don't let integrator build if throttle is too low
    integral_ripPitch = 0;
  }
  // Saturate integrator to prevent unsafe buildup
  integral_ripPitch = constrain(integral_ripPitch, -i_limit, i_limit);
  derivative_ripPitch = -ripIMU_info.GyroY;
#if defined FILTER_D
  derivative_ripPitch = biquadFilter_apply(&ripPitchDFilter, derivative_ripPitch);
#endif

  pitch_des = (Kp_ripPitch * pScaleRipPitch * error_ripPitch + Ki_ripPitch * iScaleRipPitch * integral_ripPitch +
               Kd_ripPitch * dScaleRipPitch * derivative_ripPitch);
}

void controlANGLE() {
  // DESCRIPTION: Computes control commands based on state error (angle)
  /*
   * Basic PID control to stablize on angle setpoint based on desired states
   * roll_des, pitch_des, and yaw_des computed in getDesState(). Error is simply
   * the desired state minus the actual state (ex. roll_des - roll_IMU). Two
   * safety features are implimented here regarding the I terms. The I terms are
   * saturated within specified limits on startup to prevent excessive buildup.
   * This can be seen by holding the vehicle at an angle and seeing the motors
   * ramp up on one side until they've maxed out throttle...saturating I to a
   * specified limit fixes this. The second feature defaults the I terms to 0 if
   * the throttle is at the minimum setting. This means the motors will not
   * start spooling up on the ground, and the I terms will always start from 0
   * on takeoff. This function updates the variables roll_PID, pitch_PID, and
   * yaw_PID which can be thought of as 1-D stablized signals. They are mixed to
   * the configuration of the vehicle in controlMixer().
   */

  // --- Roll --- //
  error_roll = roll_des - quadIMU_info.roll_IMU;

  integral_roll = integral_roll_prev + error_roll * dt;
  if (channel_1_pwm < 1060) { // Don't let integrator build if throttle is too low
    integral_pitch = 0;
  }
  integral_roll =
      constrain(integral_roll, -i_limit, i_limit); // Saturate integrator to prevent unsafe buildup integral_roll =
                                                   // biquadFilter_apply(&iFilter, integral_roll);
  derivative_roll = quadIMU_info.GyroX;
#if defined USE_RIP
  if (irisFlag) {
    roll_PID = 0.01 * (Kp_roll_angleFree * pScaleRoll * error_roll + Ki_roll_angleFree * iScaleRoll * integral_roll -
                       Kd_roll_angleFree * dScaleRoll * derivative_roll); // Scaled by .01 to bring within -1 to 1 range
  } else {
    roll_PID = 0.01 * (Kp_roll_angle * pScaleRoll * error_roll + Ki_roll_angle * iScaleRoll * integral_roll -
                       Kd_roll_angle * dScaleRoll * derivative_roll); // Scaled by .01 to bring within -1 to 1 range
  }
#else
  roll_PID = 0.01 * (Kp_roll_angle * pScaleRoll * error_roll + Ki_roll_angle * iScaleRoll * integral_roll -
                     Kd_roll_angle * dScaleRoll * derivative_roll); // Scaled by .01 to bring within -1 to 1 range
#endif

  // --- Pitch --- //
  error_pitch = pitch_des - quadIMU_info.pitch_IMU;

  integral_pitch = integral_pitch_prev + error_pitch * dt;
  if (channel_1_pwm < 1060) { // Don't let integrator build if throttle is too low
    integral_pitch = 0;
  }
  integral_pitch = constrain(integral_pitch, -i_limit, i_limit); // Saturate integrator to prevent unsafe buildup
                                                                 // integral_pitch = biquadFilter_apply(&iFilter,
                                                                 // integral_pitch);
  derivative_pitch = quadIMU_info.GyroY;

#if defined USE_RIP
  if (irisFlag) {
    pitch_PID =
        0.01 * (Kp_pitch_angleFree * pScalePitch * error_pitch + Ki_pitch_angleFree * iScalePitch * integral_pitch -
                Kd_pitch_angleFree * dScalePitch * derivative_pitch); // Scaled by .01 to bring within -1 to 1 range
  } else {
    pitch_PID = 0.01 * (Kp_pitch_angle * pScalePitch * error_pitch + Ki_pitch_angle * iScalePitch * integral_pitch -
                        Kd_pitch_angle * dScalePitch * derivative_pitch); // Scaled by .01 to bring within -1 to 1 range
  }
#else
  pitch_PID = 0.01 * (Kp_pitch_angle * pScalePitch * error_pitch + Ki_pitch_angle * iScalePitch * integral_pitch -
                      Kd_pitch_angle * dScalePitch * derivative_pitch); // Scaled by .01 to bring within -1 to 1 range
#endif

  // --- Yaw, stablize on rate from GyroZ --- //
  error_yaw = yaw_des - quadIMU_info.GyroZ;

  integral_yaw = integral_yaw_prev + error_yaw * dt;
  if (channel_1_pwm < 1060) { // Don't let integrator build if throttle is too low
    integral_pitch = 0;
  }
  integral_yaw = constrain(integral_yaw, -i_limit,
                           i_limit); // Saturate integrator to prevent unsafe buildup

  derivative_yaw = (error_yaw - error_yaw_prev) / dt;

  yaw_PID = 0.01 * (Kp_yaw * pScaleYaw * error_yaw + Ki_yaw * iScaleYaw * integral_yaw +
                    Kd_yaw * dScaleYaw * derivative_yaw); // Scaled by .01 to bring within -1 to 1 range

  // Update roll variables
  integral_roll_prev = integral_roll;
  // Update pitch variables
  integral_pitch_prev = integral_pitch;
  // Update yaw variables
  error_yaw_prev = error_yaw;
  integral_yaw_prev = integral_yaw;
}

// void controlANGLE2() {
//   // DESCRIPTION: Computes control commands based on state error (angle) in
//   // cascaded scheme
//   /*
//    * Gives better performance than controlANGLE() but requires much more tuning.
//    * Not reccommended for first-time setup. See the documentation for tuning
//    * this controller.
//    */
//   // Outer loop - PID on angle
//   float roll_des_ol, pitch_des_ol;
//   // Roll
//   error_roll = roll_des - roll_IMU;
//   integral_roll_ol = integral_roll_prev_ol + error_roll * dt;
//   if (channel_1_pwm < 1060) { // Don't let integrator build if throttle is too low
//     integral_roll_ol = 0;
//   }
//   integral_roll_ol = constrain(integral_roll_ol, -i_limit,
//                                i_limit); // Saturate integrator to prevent unsafe buildup
//   derivative_roll = (roll_IMU - roll_IMU_prev) / dt;
//   roll_des_ol = Kp_roll_angle * error_roll + Ki_roll_angle * integral_roll_ol; // - Kd_roll_angle*derivative_roll;
//
//   // Pitch
//   error_pitch = pitch_des - pitch_IMU;
//   integral_pitch_ol = integral_pitch_prev_ol + error_pitch * dt;
//   if (channel_1_pwm < 1060) { // Don't let integrator build if throttle is too low
//     integral_pitch_ol = 0;
//   }
//   integral_pitch_ol = constrain(integral_pitch_ol, -i_limit,
//                                 i_limit); // saturate integrator to prevent unsafe buildup
//   derivative_pitch = (pitch_IMU - pitch_IMU_prev) / dt;
//   pitch_des_ol =
//       Kp_pitch_angle * error_pitch + Ki_pitch_angle * integral_pitch_ol; // - Kd_pitch_angle*derivative_pitch;
//
//   // Apply loop gain, constrain, and LP filter for artificial damping
//   float Kl = 30.0;
//   roll_des_ol = Kl * roll_des_ol;
//   pitch_des_ol = Kl * pitch_des_ol;
//   roll_des_ol = constrain(roll_des_ol, -240.0, 240.0);
//   pitch_des_ol = constrain(pitch_des_ol, -240.0, 240.0);
//   roll_des_ol = (1.0 - B_loop_roll) * roll_des_prev + B_loop_roll * roll_des_ol;
//   pitch_des_ol = (1.0 - B_loop_pitch) * pitch_des_prev + B_loop_pitch * pitch_des_ol;
//
//   // Inner loop - PID on rate
//   // Roll
//   error_roll = roll_des_ol - GyroX;
//   integral_roll_il = integral_roll_prev_il + error_roll * dt;
//   if (channel_1_pwm < 1060) { // Don't let integrator build if throttle is too low
//     integral_roll_il = 0;
//   }
//   integral_roll_il = constrain(integral_roll_il, -i_limit,
//                                i_limit); // Saturate integrator to prevent unsafe buildup
//   derivative_roll = (error_roll - error_roll_prev) / dt;
//   roll_PID = .01 * (Kp_roll_rate * error_roll + Ki_roll_rate * integral_roll_il +
//                     Kd_roll_rate * derivative_roll); // Scaled by .01 to bring within -1 to 1 range
//
//   // Pitch
//   error_pitch = pitch_des_ol - GyroY;
//   integral_pitch_il = integral_pitch_prev_il + error_pitch * dt;
//   if (channel_1_pwm < 1060) { // Don't let integrator build if throttle is too low
//     integral_pitch_il = 0;
//   }
//   integral_pitch_il = constrain(integral_pitch_il, -i_limit,
//                                 i_limit); // Saturate integrator to prevent unsafe buildup
//   derivative_pitch = (error_pitch - error_pitch_prev) / dt;
//   pitch_PID = .01 * (Kp_pitch_rate * error_pitch + Ki_pitch_rate * integral_pitch_il +
//                      Kd_pitch_rate * derivative_pitch); // Scaled by .01 to bring within -1 to 1 range
//
//   // Yaw
//   error_yaw = yaw_des - GyroZ;
//   integral_yaw = integral_yaw_prev + error_yaw * dt;
//   if (channel_1_pwm < 1060) { // Don't let integrator build if throttle is too low
//     integral_yaw = 0;
//   }
//   integral_yaw = constrain(integral_yaw, -i_limit,
//                            i_limit); // Saturate integrator to prevent unsafe buildup
//   derivative_yaw = (error_yaw - error_yaw_prev) / dt;
//   yaw_PID = .01 * (Kp_yaw * error_yaw + Ki_yaw * integral_yaw +
//                    Kd_yaw * derivative_yaw); // Scaled by .01 to bring within -1 to 1 range
//
//   // Update roll variables
//   integral_roll_prev_ol = integral_roll_ol;
//   integral_roll_prev_il = integral_roll_il;
//   error_roll_prev = error_roll;
//   roll_IMU_prev = roll_IMU;
//   roll_des_prev = roll_des_ol;
//   // Update pitch variables
//   integral_pitch_prev_ol = integral_pitch_ol;
//   integral_pitch_prev_il = integral_pitch_il;
//   error_pitch_prev = error_pitch;
//   pitch_IMU_prev = pitch_IMU;
//   pitch_des_prev = pitch_des_ol;
//   // Update yaw variables
//   error_yaw_prev = error_yaw;
//   integral_yaw_prev = integral_yaw;
// }
//
// void controlRATE() {
//   // DESCRIPTION: Computes control commands based on state error (rate)
//   /*
//    * See explanation for controlANGLE(). Everything is the same here except the
//    * error is now the desired rate - raw gyro reading.
//    */
//   // Roll
//   error_roll = roll_des - GyroX;
//   integral_roll = integral_roll_prev + error_roll * dt;
//   if (channel_1_pwm < 1060) { // Don't let integrator build if throttle is too low
//     integral_roll = 0;
//   }
//   integral_roll = constrain(integral_roll, -i_limit,
//                             i_limit); // Saturate integrator to prevent unsafe buildup
//   derivative_roll = (error_roll - error_roll_prev) / dt;
//   roll_PID = .01 * (Kp_roll_rate * error_roll + Ki_roll_rate * integral_roll +
//                     Kd_roll_rate * derivative_roll); // Scaled by .01 to bring within -1 to 1 range
//
//   // Pitch
//   error_pitch = pitch_des - GyroY;
//   integral_pitch = integral_pitch_prev + error_pitch * dt;
//   if (channel_1_pwm < 1060) { // Don't let integrator build if throttle is too low
//     integral_pitch = 0;
//   }
//   integral_pitch = constrain(integral_pitch, -i_limit,
//                              i_limit); // Saturate integrator to prevent unsafe buildup
//   derivative_pitch = (error_pitch - error_pitch_prev) / dt;
//   pitch_PID = .01 * (Kp_pitch_rate * error_pitch + Ki_pitch_rate * integral_pitch +
//                      Kd_pitch_rate * derivative_pitch); // Scaled by .01 to bring within -1 to 1 range
//
//   // Yaw, stablize on rate from GyroZ
//   error_yaw = yaw_des - GyroZ;
//   integral_yaw = integral_yaw_prev + error_yaw * dt;
//   if (channel_1_pwm < 1060) { // Don't let integrator build if throttle is too low
//     integral_yaw = 0;
//   }
//   integral_yaw = constrain(integral_yaw, -i_limit,
//                            i_limit); // Saturate integrator to prevent unsafe buildup
//   derivative_yaw = (error_yaw - error_yaw_prev) / dt;
//   yaw_PID = .01 * (Kp_yaw * error_yaw + Ki_yaw * integral_yaw +
//                    Kd_yaw * derivative_yaw); // Scaled by .01 to bring within -1 to 1 range
//
//   // Update roll variables
//   error_roll_prev = error_roll;
//   integral_roll_prev = integral_roll;
//   GyroX_prev = GyroX;
//   // Update pitch variables
//   error_pitch_prev = error_pitch;
//   integral_pitch_prev = integral_pitch;
//   GyroY_prev = GyroY;
//   // Update yaw variables
//   error_yaw_prev = error_yaw;
//   integral_yaw_prev = integral_yaw;
// }

void scaleCommands() {
  // DESCRIPTION: Scale normalized actuator commands to values for ESC/Servo
  // protocol
  /*
   * mX_command_scaled variables from the mixer function are scaled to 125-250us
   * for OneShot125 protocol. sX_command_scaled variables from the mixer
   * function are scaled to 0-180 for the servo library using standard PWM.
   * mX_command_PWM are updated here which are used to command the motors in
   * commandMotors(). sX_command_PWM are updated which are used to command the
   * servos.
   */
  // Scaled to 125us - 250us for oneshot125 protocol
  m1_command_PWM = m1_command_scaled * 125 + 125;
  m2_command_PWM = m2_command_scaled * 125 + 125;
  m3_command_PWM = m3_command_scaled * 125 + 125;
  m4_command_PWM = m4_command_scaled * 125 + 125;
  m5_command_PWM = m5_command_scaled * 125 + 125;
  m6_command_PWM = m6_command_scaled * 125 + 125;
  // Constrain commands to motors within oneshot125 bounds
  m1_command_PWM = constrain(m1_command_PWM, 125, 250);
  m2_command_PWM = constrain(m2_command_PWM, 125, 250);
  m3_command_PWM = constrain(m3_command_PWM, 125, 250);
  m4_command_PWM = constrain(m4_command_PWM, 125, 250);
  m5_command_PWM = constrain(m5_command_PWM, 125, 250);
  m6_command_PWM = constrain(m6_command_PWM, 125, 250);
  // Scaled to 0-180 for servo library
  s1_command_PWM = s1_command_scaled * 180;
  s2_command_PWM = s2_command_scaled * 180;
  s3_command_PWM = s3_command_scaled * 180;
  s4_command_PWM = s4_command_scaled * 180;
  s5_command_PWM = s5_command_scaled * 180;
  s6_command_PWM = s6_command_scaled * 180;
  s7_command_PWM = s7_command_scaled * 180;

  // Constrain commands to servos within servo library bounds
  s1_command_PWM = constrain(s1_command_PWM, 0, 180);
  s2_command_PWM = constrain(s2_command_PWM, 0, 180);
  s3_command_PWM = constrain(s3_command_PWM, 0, 180);
  s4_command_PWM = constrain(s4_command_PWM, 0, 180);
  s5_command_PWM = constrain(s5_command_PWM, 0, 180);
  s6_command_PWM = constrain(s6_command_PWM, 0, 180);
  s7_command_PWM = constrain(s7_command_PWM, 0, 180);
}

void getCommands() {
  // DESCRIPTION: Get raw PWM values for every channel from the radio
  /*
   * Updates radio PWM commands in loop based on current available commands.
   * channel_x_pwm is the raw command used in the rest of the loop. If using a
   * PWM or PPM receiver, the radio commands are retrieved from a function in
   * the readPWM file separate from this one which is running a bunch of
   * interrupts to continuously update the radio readings. If using an SBUS
   * receiver, the alues are pulled from the SBUS library directly. The raw
   * radio commands are filtered with a first order low-pass filter to eliminate
   * any really high frequency noise.
   */

#if defined USE_PPM_RX || defined USE_PWM_RX
  channel_1_pwm = getRadioPWM(1);
  channel_2_pwm = getRadioPWM(2);
  channel_3_pwm = getRadioPWM(3);
  channel_4_pwm = getRadioPWM(4);
  channel_5_pwm = getRadioPWM(5);
  channel_6_pwm = getRadioPWM(6);

#elif defined USE_SBUS_RX
  if (sbus.read(&sbusChannels[0], &sbusFailSafe, &sbusLostFrame)) {
    // sBus scaling below is for Taranis-Plus and X4R-SB
    float scale = 0.615;
    float bias = 895.0;
    channel_1_pwm = sbusChannels[0] * scale + bias;
    channel_2_pwm = sbusChannels[1] * scale + bias;
    channel_3_pwm = sbusChannels[2] * scale + bias;
    channel_4_pwm = sbusChannels[3] * scale + bias;
    channel_5_pwm = sbusChannels[4] * scale + bias;
    channel_6_pwm = sbusChannels[5] * scale + bias;
    channel_7_pwm = sbusChannels[6] * scale + bias;
    channel_8_pwm = sbusChannels[7] * scale + bias;
    channel_9_pwm = sbusChannels[8] * scale + bias;
    channel_10_pwm = sbusChannels[9] * scale + bias;
    channel_11_pwm = sbusChannels[10] * scale + bias;
    channel_12_pwm = sbusChannels[11] * scale + bias;
    channel_13_pwm = sbusChannels[12] * scale + bias;
    channel_14_pwm = sbusChannels[13] * scale + bias;
  }

#elif defined USE_DSM_RX
  if (DSM.timedOut(micros())) {
    // Serial.println("*** DSM RX TIMED OUT ***");
  } else if (DSM.gotNewFrame()) {
    uint16_t values[num_DSM_channels];
    DSM.getChannelValues(values, num_DSM_channels);

    channel_1_pwm = values[0];
    channel_2_pwm = values[1];
    channel_3_pwm = values[2];
    channel_4_pwm = values[3];
    channel_5_pwm = values[4];
    channel_6_pwm = values[5];
  }
#endif

  // Low-pass the critical commands and update previous values
  float b = 0.7; // Lower=slower, higher=noiser
  channel_1_pwm = (1.0 - b) * channel_1_pwm_prev + b * channel_1_pwm;
  channel_2_pwm = (1.0 - b) * channel_2_pwm_prev + b * channel_2_pwm;
  channel_3_pwm = (1.0 - b) * channel_3_pwm_prev + b * channel_3_pwm;
  channel_4_pwm = (1.0 - b) * channel_4_pwm_prev + b * channel_4_pwm;

  // Update prev values
  channel_1_pwm_prev = channel_1_pwm;
  channel_2_pwm_prev = channel_2_pwm;
  channel_3_pwm_prev = channel_3_pwm;
  channel_4_pwm_prev = channel_4_pwm;
}

void failSafe() {
  // DESCRIPTION: If radio gives garbage values, set all commands to default
  // values
  /*
   * Radio connection failsafe used to check if the getCommands() function is
   * returning acceptable pwm values. If any of the commands are lower than 800
   * or higher than 2200, then we can be certain that there is an issue with the
   * radio connection (most likely hardware related). If any of the channels
   * show this failure, then all of the radio commands channel_x_pwm are set to
   * default failsafe values specified in the setup. Comment out this function
   * when troubleshooting your radio connection in case any extreme values are
   * triggering this function to overwrite the printed variables.
   */
  int minVal = 800;
  int maxVal = 2200;
  int check1 = 0;
  int check2 = 0;
  int check3 = 0;
  int check4 = 0;
  int check5 = 0;
  int check6 = 0;
  failureFlag = 0;

  // Triggers for failure criteria
  if (channel_1_pwm > maxVal || channel_1_pwm < minVal)
    check1 = 1;
  if (channel_2_pwm > maxVal || channel_2_pwm < minVal)
    check2 = 1;
  if (channel_3_pwm > maxVal || channel_3_pwm < minVal)
    check3 = 1;
  if (channel_4_pwm > maxVal || channel_4_pwm < minVal)
    check4 = 1;
  if (channel_5_pwm > maxVal || channel_5_pwm < minVal)
    check5 = 1;
  if (channel_6_pwm > maxVal || channel_6_pwm < minVal)
    check6 = 1;

  // If any failures, set to default failsafe values
  if ((check1 + check2 + check3 + check4 + check5 + check6) > 0) {
    channel_1_pwm = channel_1_fs;
    channel_2_pwm = channel_2_fs;
    channel_3_pwm = channel_3_fs;
    channel_4_pwm = channel_4_fs;
    channel_5_pwm = channel_5_fs;
    channel_6_pwm = channel_6_fs;
    channel_7_pwm = channel_7_fs;
    channel_8_pwm = channel_8_fs;
    channel_9_pwm = channel_9_fs;
    channel_10_pwm = channel_10_fs;
    channel_11_pwm = channel_11_fs;
    channel_12_pwm = channel_12_fs;
    channel_13_pwm = channel_13_fs;
    channel_14_pwm = channel_14_fs;

    failureFlag = 1;
  }
}

void m1_EndPulse() {
  digitalWrite(m1Pin, LOW);
  m1_writing = false;
}

void m2_EndPulse() {
  digitalWrite(m2Pin, LOW);
  m2_writing = false;
}

void m3_EndPulse() {
  digitalWrite(m3Pin, LOW);
  m3_writing = false;
}

void m4_EndPulse() {
  digitalWrite(m4Pin, LOW);
  m4_writing = false;
}

void commandMotors() {
  // Serial.print("M1: ");
  // Serial.print(m1_command_PWM);
  // Serial.print(" M2: ");
  // Serial.print(m2_command_PWM);
  // Serial.print(" M3: ");
  // Serial.print(m3_command_PWM);
  // Serial.print(" M4: ");
  // Serial.println(m4_command_PWM);

  digitalWrite(m1Pin, HIGH);
  m1_writing = true;
  m1_timer.trigger(m1_command_PWM);
  digitalWrite(m2Pin, HIGH);
  m2_writing = true;
  m2_timer.trigger(m2_command_PWM);
  digitalWrite(m3Pin, HIGH);
  m3_writing = true;
  m3_timer.trigger(m3_command_PWM);
  digitalWrite(m4Pin, HIGH);
  m4_writing = true;
  m4_timer.trigger(m4_command_PWM);
}

void armMotors() {
  // DESCRIPTION: Sends many command pulses to the motors, to be used to arm
  // motors in the void setup()
  /*
   *  Loops over the commandMotors() function 1500 times with a delay in between,
   * simulating how the commandMotors() function is used in the main loop.
   * Ensures motors arm within the void setup() where there are some delays for
   * other processes that sometimes prevent motors from arming.
   */
  for (int i = 0; i <= 10000; i++) {
    commandMotors();
    delayMicroseconds(500);
  }
}

void calibrateESCs() {
  // DESCRIPTION: Used in void setup() to allow standard ESC calibration
  // procedure with the radio to take place.
  /*
   *  Simulates the void loop(), but only for the purpose of providing throttle
   * pass through to the motors, so that you can power up with throttle at full,
   * let ESCs begin arming sequence, and lower throttle to zero. This function
   * should only be uncommented when performing an ESC calibration.
   */
  while (true) {
    prev_time = current_time;
    current_time = micros();
    dt = (current_time - prev_time) / 1000000.0;

    digitalWrite(13, HIGH); // LED on to indicate we are not in main loop

    getCommands();                       // Pulls current available radio commands
    failSafe();                          // Prevent failures in event of bad receiver connection,
                                         // defaults to failsafe values assigned in setup
    getDesState();                       // Convert raw commands to normalized values based on
                                         // saturated control limits
    getIMUData(&quadIMU_info, &quadIMU); // Pulls raw gyro, accelerometer, and magnetometer data from
                                         // IMU and LP filters to remove noise
    Madgwick6DOF(&quadIMU_info, dt);
    getDesState(); // Convert raw commands to normalized values based on
                   // saturated control limits

    m1_command_scaled = thro_des;
    m2_command_scaled = thro_des;
    m3_command_scaled = thro_des;
    m4_command_scaled = thro_des;
    s1_command_scaled = thro_des;
    s2_command_scaled = thro_des;
    s3_command_scaled = thro_des;
    s4_command_scaled = thro_des;
    s5_command_scaled = thro_des;
    s6_command_scaled = thro_des;
    s7_command_scaled = thro_des;
    scaleCommands(); // Scales motor commands to 125 to 250 range (oneshot125
                     // protocol) and servo PWM commands to 0 to 180 (for servo
                     // library)

    // throttleCut(); //Directly sets motor commands to low based on state of ch5

    commandMotors();
    servo1.write(s1_command_PWM);
    servo2.write(s2_command_PWM);
    servo3.write(s3_command_PWM);
    servo4.write(s4_command_PWM);
    servo5.write(s5_command_PWM);
    servo6.write(s6_command_PWM);
    servo7.write(s7_command_PWM);

    // printRadioData(); //Radio pwm values (expected: 1000 to 2000)

    loopRate(2000); // Do not exceed 2000Hz, all filter parameters tuned to
                    // 2000Hz by default
  }
}

float floatFaderLinear(float param, float param_min, float param_max, float fadeTime, int state, int loopFreq) {
  // DESCRIPTION: Linearly fades a float type variable between min and max
  // bounds based on desired high or low state and time
  /*
   *  Takes in a float variable, desired minimum and maximum bounds, fade time,
   * high or low desired state, and the loop frequency and linearly interpolates
   * that param variable between the maximum and minimum bounds. This function
   * can be called in controlMixer() and high/low states can be determined by
   * monitoring the state of an auxillarly radio channel. For example, if
   * channel_6_pwm is being monitored to switch between two dynamic
   * configurations (hover and forward flight), this function can be called
   * within the logical statements in order to fade controller gains, for
   * example between the two dynamic configurations. The 'state' (1 or 0) can be
   * used to designate the two final options for that control gain based on the
   * dynamic configuration assignment to the auxillary radio channel.
   *
   */
  float diffParam = (param_max - param_min) / (fadeTime * loopFreq); // Difference to add or subtract from param for
                                                                     // each loop iteration for desired fadeTime

  if (state == 1) { // Maximum param bound desired, increase param by diffParam
                    // for each loop iteration
    param = param + diffParam;
  } else if (state == 0) { // Minimum param bound desired, decrease param by
                           // diffParam for each loop iteration
    param = param - diffParam;
  }

  param = constrain(param, param_min,
                    param_max); // Constrain param within max bounds

  return param;
}

float floatFaderLinear2(float param, float param_des, float param_lower, float param_upper, float fadeTime_up,
                        float fadeTime_down, int loopFreq) {
  // DESCRIPTION: Linearly fades a float type variable from its current value to
  // the desired value, up or down
  /*
   *  Takes in a float variable to be modified, desired new position, upper
   * value, lower value, fade time, and the loop frequency and linearly fades
   * that param variable up or down to the desired value. This function can be
   * called in controlMixer() to fade up or down between flight modes monitored
   * by an auxillary radio channel. For example, if channel_6_pwm is being
   *  monitored to switch between two dynamic configurations (hover and forward
   * flight), this function can be called within the logical statements in order
   * to fade controller gains, for example between the two dynamic
   * configurations.
   *
   */
  if (param > param_des) { // Need to fade down to get to desired
    float diffParam = (param_upper - param_des) / (fadeTime_down * loopFreq);
    param = param - diffParam;
  } else if (param < param_des) { // Need to fade up to get to desired
    float diffParam = (param_des - param_lower) / (fadeTime_up * loopFreq);
    param = param + diffParam;
  }

  param = constrain(param, param_lower,
                    param_upper); // Constrain param within max bounds

  return param;
}

void switchRollYaw(int reverseRoll, int reverseYaw) {
  // DESCRIPTION: Switches roll_des and yaw_des variables for tailsitter-type
  // configurations
  /*
   * Takes in two integers (either 1 or -1) corresponding to the desired
   * reversing of the roll axis and yaw axis, respectively. Reversing of the
   * roll or yaw axis may be needed when switching between the two for some
   * dynamic configurations. Inputs of 1, 1 does not reverse either of them,
   * while -1, 1 will reverse the output corresponding to the new roll axis.
   * This function may be replaced in the future by a function that switches the
   * IMU data instead (so that angle can also be estimated with the IMU tilted
   * 90 degrees from default level).
   */
  float switch_holder;

  switch_holder = yaw_des;
  yaw_des = reverseYaw * roll_des;
  roll_des = reverseRoll * switch_holder;
}

int throttleCut() {
  // DESCRIPTION: Directly set actuator outputs to minimum value if triggered
  /*
   * Monitors the state of radio command channel_5_pwm and directly sets the
   * mx_command_PWM values to minimum (120 is minimum for oneshot125 protocol, 0
   * is minimum for standard PWM servo library used) if channel 5 is high. This
   * is the last function called before commandMotors() is called so that the
   * last thing checked is if the user is giving permission to command the
   * motors to anything other than minimum value. Safety first.
   */
  if (channel_5_pwm > 1500) {
    m1_command_PWM = 125;
    m2_command_PWM = 125;
    m3_command_PWM = 125;
    m4_command_PWM = 125;
    s1_command_PWM = 0;
    s2_command_PWM = 0;
    s3_command_PWM = 0;
    s4_command_PWM = 0;
    s5_command_PWM = 0;
    s6_command_PWM = 0;
    s7_command_PWM = 0;
    return 1;
  }
  return 0;
}

void calibrateMagnetometer() {
#if defined USE_MPU9250_SPI
  float success;
  Serial.println("Beginning magnetometer calibration in");
  Serial.println("3...");
  delay(1000);
  Serial.println("2...");
  delay(1000);
  Serial.println("1...");
  delay(1000);
  Serial.println("Rotate the IMU about all axes until complete.");
  Serial.println(" ");
  success = mpu9250.calibrateMag();
  if (success) {
    Serial.println("Calibration Successful!");
    Serial.println("Please comment out the calibrateMagnetometer() function "
                   "and copy these values into the code:");
    Serial.print("float MagErrorX = ");
    Serial.print(mpu9250.getMagBiasX_uT());
    Serial.println(";");
    Serial.print("float MagErrorY = ");
    Serial.print(mpu9250.getMagBiasY_uT());
    Serial.println(";");
    Serial.print("float MagErrorZ = ");
    Serial.print(mpu9250.getMagBiasZ_uT());
    Serial.println(";");
    Serial.print("float MagScaleX = ");
    Serial.print(mpu9250.getMagScaleFactorX());
    Serial.println(";");
    Serial.print("float MagScaleY = ");
    Serial.print(mpu9250.getMagScaleFactorY());
    Serial.println(";");
    Serial.print("float MagScaleZ = ");
    Serial.print(mpu9250.getMagScaleFactorZ());
    Serial.println(";");
    Serial.println(" ");
    Serial.println("If you are having trouble with your attitude estimate at a "
                   "new flying location, repeat this process as needed.");
  } else {
    Serial.println("Calibration Unsuccessful. Please reset the board and try again.");
  }

  while (1)
    ; // Halt code so it won't enter main loop until this function commented out
#endif
  Serial.println("Error: MPU9250 not selected. Cannot calibrate non-existent "
                 "magnetometer.");
  while (1)
    ; // Halt code so it won't enter main loop until this function commented out
}

void loopRate(int freq) {
  // DESCRIPTION: Regulate main loop rate to specified frequency in Hz
  /*
   * It's good to operate at a constant loop rate for filters to remain stable
   * and whatnot. Interrupt routines running in the background cause the loop
   * rate to fluctuate. This function basically just waits at the end of every
   * loop iteration until the correct time has passed since the start of the
   * current loop for the desired loop rate in Hz. 2kHz is a good rate to be at
   * because the loop nominally will run between 2.8kHz - 4.2kHz. This lets us
   * have a little room to add extra computations and remain above 2kHz, without
   * needing to retune all of our filtering parameters.
   */
  float invFreq = 1.0 / freq * 1000000.0;
  unsigned long checker = micros();

  // Sit in loop until appropriate time has passed
  while (invFreq > (checker - current_time)) {
    checker = micros();
  }
}

void loopBlink() {
  // DESCRIPTION: Blink LED on board to indicate main loop is running
  /*
   * It looks cool.
   */
  if (current_time - blink_counter > blink_delay) {
    blink_counter = micros();
    digitalWrite(13, blinkAlternate); // Pin 13 is built in LED

    if (blinkAlternate == 1) {
      blinkAlternate = 0;
      blink_delay = 100000;
    } else if (blinkAlternate == 0) {
      blinkAlternate = 1;
      blink_delay = 2000000;
    }
  }
}

void setupBlink(int numBlinks, int upTime, int downTime) {
  // DESCRIPTION: Simple function to make LED on board blink as desired
  for (int j = 1; j <= numBlinks; j++) {
    digitalWrite(13, LOW);
    delay(downTime);
    digitalWrite(13, HIGH);
    delay(upTime);
  }
}

void getJoyAngle() {
  // Read the raw analog values (0 to 1023)
  joyRollCounts = analogRead(joyRollPin);
  joyPitchCounts = analogRead(joyPitchPin);

  joyRoll = (static_cast<float>(joyRollCounts) - fullRange_joyRoll_half - joyRollCounts_min) / fullRange_joyRoll *
                (joyRollAngle_min - joyRollAngle_max) +
            joyRollOffset;
  joyPitch = (static_cast<float>(joyPitchCounts) - fullRange_joyPitch_half - joyPitchCounts_min) / fullRange_joyPitch *
                 (joyPitchAngle_min - joyPitchAngle_max) +
             joyPitchOffset;

  // Determine ripRoll and ripPitch in the inertial frame based on joystick angle
  ripRoll = joyRoll + quadIMU_info.roll_IMU;
  ripPitch = joyPitch + quadIMU_info.pitch_IMU;
}

void openIris() {
  iris.write(70);
  servoLoopCounter = 0;
}

void closeIris() {
  if (servoLoopCounter < 500) {
    iris.write(146);
    servoLoopCounter++;
  } else {
    iris.write(146);
  }
}

void calibrateJoystick() {
  joyRollCounts_max = 0;
  joyRollCounts_min = 1000;
  joyPitchCounts_max = 0;
  joyPitchCounts_min = 1000;

  while (1) {
    joyRollCounts = analogRead(joyRollPin);
    joyPitchCounts = analogRead(joyPitchPin);

    if (joyRollCounts < joyRollCounts_min) {
      joyRollCounts_min = joyRollCounts;
    }
    if (joyRollCounts > joyRollCounts_max) {
      joyRollCounts_max = joyRollCounts;
    }
    if (joyPitchCounts < joyPitchCounts_min) {
      joyPitchCounts_min = joyPitchCounts;
    }
    if (joyPitchCounts > joyPitchCounts_max) {
      joyPitchCounts_max = joyPitchCounts;
    }
    Serial.print("joyRollCounts_max = ");
    Serial.print(joyRollCounts_max);
    Serial.print("\t");
    Serial.print("joyRollCounts_min = ");
    Serial.print(joyRollCounts_min);
    Serial.print("\t");
    Serial.print("joyPitchCounts_max = ");
    Serial.print(joyPitchCounts_max);
    Serial.print("\t");
    Serial.print("joyRollCounts_min = ");
    Serial.println(joyPitchCounts_min);
  }
}

void getPScale() {
  float scaleVal;
#ifdef TUNE_RIP
  scaleVal = 1.0f + (channel_10_pwm - 1500.0f) / 1000.0f * 0.25f;
  if (scaleVal < 0.0f) {
    scaleVal = 0.0f;
  }
  pScaleRipRoll = scaleVal;
  pScaleRipPitch = scaleVal;
#elif defined TUNE_CORE
  scaleVal = 1.0f + (channel_10_pwm - 1000.0f) / 1000.0f * 1.0f;
  if (scaleVal < 0.0f) {
    scaleVal = 0.0f;
  }
  pScaleRoll = scaleVal;
  pScalePitch = scaleVal;
#endif
}

void getDScale() {
  float scaleVal;
#ifdef TUNE_RIP
  scaleVal = 1.0f + (channel_12_pwm - 1500.0f) / 1000.0f * 0.25f;
  if (scaleVal < 0.0f) {
    scaleVal = 0.0f;
  }
  dScaleRipRoll = scaleVal;
  dScaleRipPitch = scaleVal;
#elif defined TUNE_CORE
  scaleVal = 1.0f + (channel_12_pwm - 1000.0f) / 1000.0f * 2.0f;
  if (scaleVal < 0.0f) {
    scaleVal = 0.0f;
  }
  dScaleRoll = scaleVal;
  dScalePitch = scaleVal;
#endif
}

void getIScale() {
  float scaleVal;
#ifdef TUNE_RIP
  scaleVal = 1.0f + (channel_11_pwm - 1500.0f) / 1000.0f * 0.25f;
  if (scaleVal < 0.0f) {
    scaleVal = 0.0f;
  }
  iScaleRipRoll = scaleVal;
  iScaleRipPitch = scaleVal;
#elif defined TUNE_CORE
  scaleVal = 1.0f + (channel_11_pwm - 1000.0f) / 1000.0f * 10.0f;
  if (scaleVal < 0.0f) {
    scaleVal = 0.0f;
  }
  iScaleRoll = scaleVal;
  iScalePitch = scaleVal;
#endif
}

void scaleBoth() {
  float scaleMultiplier;
  scaleMultiplier = 1.0f + (channel_13_pwm - 1015.0f) / 1000.0f * 0.25f;
#ifdef TUNE_RIP
  pScaleRipRoll *= scaleMultiplier;
  iScaleRipRoll *= scaleMultiplier;
  dScaleRipRoll *= scaleMultiplier;
  pScaleRipPitch *= scaleMultiplier;
  iScaleRipPitch *= scaleMultiplier;
  dScaleRipPitch *= scaleMultiplier;
#elif defined TUNE_CORE
  pScaleRoll *= scaleMultiplier;
  iScaleRoll *= scaleMultiplier;
  dScaleRoll *= scaleMultiplier;
  pScalePitch *= scaleMultiplier;
  iScalePitch *= scaleMultiplier;
  dScalePitch *= scaleMultiplier;
#endif
}

void ripExtremeAngleCheck() {
  if (abs(acos(cos(joyRoll * PI / 180.0f) * cos(joyPitch * PI / 180.0f))) > EXTREME_RIP_ANGLE * PI / 180.0f) {
    closeIris();
    extremeAngleFlag = 1;
  }
}
//=========================================================================================//

// HELPER FUNCTIONS
float invSqrt(float x) {
  // Fast inverse sqrt for madgwick filter
  /*
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  y = y * (1.5f - (halfx * y * y));
  return y;
  */
  /*
  //alternate form:
  unsigned int i = 0x5F1F1412 - (*(unsigned int*)&x >> 1);
  float tmp = *(float*)&i;
  float y = tmp * (1.69000231f - 0.714158168f * x * tmp * tmp)
  return y;
  */
  return 1.0 / sqrtf(x); // Teensy is fast enough to just take the compute
                         // penalty lol suck it arduino nano
}
