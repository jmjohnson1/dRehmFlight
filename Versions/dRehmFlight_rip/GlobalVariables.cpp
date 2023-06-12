#include "GlobalVariables.h"

//DECLARE GLOBAL VARIABLES

//General stuff
float dt;
unsigned long current_time, prev_time;
unsigned long print_counter, serial_counter;
unsigned long blink_counter, blink_delay;
bool blinkAlternate;
unsigned long print_counterSD = 200000;

//Radio communication:
int channel_1_pwm, channel_2_pwm, channel_3_pwm, channel_4_pwm, channel_5_pwm, channel_6_pwm,
channel_7_pwm, channel_8_pwm, channel_9_pwm, channel_10_pwm, channel_11_pwm, channel_12_pwm,
channel_13_pwm;
int channel_1_pwm_prev, channel_2_pwm_prev, channel_3_pwm_prev, channel_4_pwm_prev;
int channel_1_pwm_pre, channel_2_pwm_pre, channel_3_pwm_pre, channel_4_pwm_pre;


//IMU:
float AccX, AccY, AccZ;
float AccX_prev, AccY_prev, AccZ_prev;
float GyroX, GyroY, GyroZ;
float GyroX_prev, GyroY_prev, GyroZ_prev;
float MagX, MagY, MagZ;
float MagX_prev, MagY_prev, MagZ_prev;
float roll_IMU, pitch_IMU, yaw_IMU;
float roll_IMU_prev, pitch_IMU_prev;
float q0 = 1.0f; //Initialize quaternion for madgwick filter
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;

//Normalized desired state:
float thro_des, roll_des, pitch_des, yaw_des;
float roll_passthru, pitch_passthru, yaw_passthru;
float alphaRoll_des, betaPitch_des;

//Controller:
float roll_PID  = 0;
float pitch_PID = 0;
float yaw_PID   = 0;

//Mixer
float m1_command_scaled, m2_command_scaled, m3_command_scaled, m4_command_scaled, m5_command_scaled,
m6_command_scaled;
int m1_command_PWM, m2_command_PWM, m3_command_PWM, m4_command_PWM, m5_command_PWM, m6_command_PWM;
float s1_command_scaled, s2_command_scaled, s3_command_scaled, s4_command_scaled, s5_command_scaled,
s6_command_scaled, s7_command_scaled;
int s1_command_PWM, s2_command_PWM, s3_command_PWM, s4_command_PWM, s5_command_PWM, s6_command_PWM,
s7_command_PWM;

// Flag for whether or not Iris is open
bool irisFlag = 0;

// Joystick values
int alphaCounts;        // Joystick x-axis rotation analog signal
int betaCounts;        // Joystick y-axis rotation analog signal
float alpha;       // Joystick x-axis rotation angle
float beta;       // Joystick x-axis rotation angle
float alphaRoll;
float betaPitch;

float alphaOffset = 0.0f;
float betaOffset = 0.0f;

// Values for the setDesStateSerial() function
float serialInputValue = 0.0f;  // User input over the serial line
float sineFrequency = 0.0f; // If using sine wave, its frequency in Hz
float sineTime = 0.0f; 		// Counter used to determine time in the sine functions (seconds)

// A flag for whether or not the sine sweep should be conducted. User input while the program is
// running sets this. DON'T SET THIS YOURSELF!
bool sweepFlag = 0;     

// SD Card settings
String filePrefix = "flight_data";
String fileExtension = ".csv";
String fileName;

File dataFile;

bool SD_is_present = 0;

int cutoff_val = 150;
int maxCutCounter = 10;
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
