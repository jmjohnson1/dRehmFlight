# 1 "/home/james/Documents/dRehmFlight/Versions/dRehmFlight_quad/dRehmFlight_quad.ino"
//Arduino/Teensy Flight Controller - dRehmFlight
//Author: Nicholas Rehm
//Project Start: 1/6/2020
//Last Updated: 7/29/2022

//================================================================================================//

//CREDITS + SPECIAL THANKS
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
# 28 "/home/james/Documents/dRehmFlight/Versions/dRehmFlight_quad/dRehmFlight_quad.ino"
//================================================================================================//
//                                    USER-SPECIFIED DEFINES                        						  //                                                                 
//================================================================================================//

// Keep track of last error term
float errorOld_yaw = 0;
float errorOld_alpha = 0;
float errorOld_beta = 0;

// Keep track of last integral term
float integralOld_roll = 0;
float integralOld_pitch = 0;
float integralOld_yaw = 0;
float integralOld_alpha = 0;
float integralOld_beta = 0;

//Uncomment only one receiver type
//#define USE_PWM_RX
//#define USE_PPM_RX

//#define USE_DSM_RX
static const uint8_t num_DSM_channels = 6; //If using DSM RX, change this to match the number of 
// 																					 transmitter channels you have

//Uncomment only one IMU

//#define USE_MPU9250_SPI

//Uncomment only one full scale gyro range (deg/sec)

//#define GYRO_500DPS
//#define GYRO_1000DPS
//#define GYRO_2000DPS

//Uncomment only one full scale accelerometer range (G's)

//#define ACCEL_4G
//#define ACCEL_8G
//#define ACCEL_16G



//================================================================================================//



//REQUIRED LIBRARIES (included with download in main sketch folder)
# 76 "/home/james/Documents/dRehmFlight/Versions/dRehmFlight_quad/dRehmFlight_quad.ino" 2
# 77 "/home/james/Documents/dRehmFlight/Versions/dRehmFlight_quad/dRehmFlight_quad.ino" 2
# 78 "/home/james/Documents/dRehmFlight/Versions/dRehmFlight_quad/dRehmFlight_quad.ino" 2
# 79 "/home/james/Documents/dRehmFlight/Versions/dRehmFlight_quad/dRehmFlight_quad.ino" 2
# 80 "/home/james/Documents/dRehmFlight/Versions/dRehmFlight_quad/dRehmFlight_quad.ino" 2
# 81 "/home/james/Documents/dRehmFlight/Versions/dRehmFlight_quad/dRehmFlight_quad.ino" 2
# 82 "/home/james/Documents/dRehmFlight/Versions/dRehmFlight_quad/dRehmFlight_quad.ino" 2
const int chipSelect = 254;


# 86 "/home/james/Documents/dRehmFlight/Versions/dRehmFlight_quad/dRehmFlight_quad.ino" 2







# 94 "/home/james/Documents/dRehmFlight/Versions/dRehmFlight_quad/dRehmFlight_quad.ino" 2
  MPU6050 mpu6050;
# 104 "/home/james/Documents/dRehmFlight/Versions/dRehmFlight_quad/dRehmFlight_quad.ino"
//================================================================================================//



//Setup gyro and accel full scale value selection and scale factor
# 160 "/home/james/Documents/dRehmFlight/Versions/dRehmFlight_quad/dRehmFlight_quad.ino"
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
float B_madgwick = 0.04; //Madgwick filter parameter
float B_accel = 0.14; //Accelerometer LP filter paramter, (MPU6050 default: 0.14. MPU9250 default: 0.2)
float B_gyro = 0.1; //Gyro LP filter paramter, (MPU6050 default: 0.1. MPU9250 default: 0.17)
float B_mag = 1.0; //Magnetometer LP filter parameter

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
float AccErrorX = 0.05;
float AccErrorY = -0.02;
float AccErrorZ = -0.05;
float GyroErrorX = -3.02;
float GyroErrorY = -0.50;
float GyroErrorZ = -0.32;
//Controller parameters (take note of defaults before modifying!): 
//Integrator saturation level, mostly for safety (default 25.0)
float i_limit = 25.0;
//Max roll/pitch angles in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode 
float maxRoll = 30.0;
float maxPitch = 30.0;
//Max yaw rate in deg/sec
float maxYaw = 160.0;

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
//Roll damping term for controlANGLE2(), lower is more damping (must be between 0 to 1)
float B_loop_roll = 0.9;
//Pitch damping term for controlANGLE2(), lower is more damping (must be between 0 to 1)
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

// JOYSTICK ANALOG INPUT RANGES //
int alphaCounts_min = 148;
int alphaCounts_max = 1023;
int betaCounts_min = 51;
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
float maxFreq = 1.5f; // Maximum frequency of the sine sweep in Hz
float minFreq = 0.05f; // Minimum frequency of the sine sweep in Hz
float sweepTime = 60; // How long to run the sweep for in seconds


//================================================================================================//
//                                      DECLARE PINS                                              //
//================================================================================================//                                          

//NOTE: Pin 13 is reserved for onboard LED, pins 18 and 19 are reserved for the MPU6050 IMU for 
// 			default setup
//Radio note:
// 			If using SBUS, connect to pin 21 (RX5), if using DSM, connect to pin 15 (RX3)
const int ch1Pin = 15; //throttle
const int ch2Pin = 16; //ail
const int ch3Pin = 17; //ele
const int ch4Pin = 20; //rudd
const int ch5Pin = 21; //gear (throttle cut)
const int ch6Pin = 22; //aux1 (free aux channel)
const int PPM_Pin = 23;
//OneShot125 ESC pin outputs:
const int m1Pin = 0;
const int m2Pin = 1;
const int m3Pin = 2;
const int m4Pin = 3;
const int m5Pin = 4;
const int m6Pin = 5;
//PWM servo or ESC outputs:
const int servo1Pin = 6;
const int servo2Pin = 7;
const int servo3Pin = 10;
const int servo4Pin = 9;
const int servo5Pin = 8;
const int servo6Pin = 11;
const int servo7Pin = 12;

// Joystick pins
const int joyAlphaPin = 41;
const int joyBetaPin = 40;

// Pin and object for iris servo:
const int irisPin = 24;
PWMServo iris;

//Create servo objects to control a servo or ESC with PWM
PWMServo servo1;
PWMServo servo2;
PWMServo servo3;
PWMServo servo4;
PWMServo servo5;
PWMServo servo6;
PWMServo servo7;

//================================================================================================//

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


  SBUS sbus(Serial5);
  uint16_t sbusChannels[16];
  bool sbusFailSafe;
  bool sbusLostFrame;





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
float alpha_des, beta_des;

//Controller:
float error_roll, error_roll_prev, roll_des_prev, integral_roll, integral_roll_il, integral_roll_ol,
   integral_roll_prev, integral_roll_prev_il, integral_roll_prev_ol, derivative_roll,
   roll_PID = 0;
float error_pitch, error_pitch_prev, pitch_des_prev, integral_pitch, integral_pitch_il,
   integral_pitch_ol, integral_pitch_prev, integral_pitch_prev_il, integral_pitch_prev_ol,
   derivative_pitch, pitch_PID = 0;
float error_yaw, error_yaw_prev, integral_yaw, integral_yaw_prev, derivative_yaw, yaw_PID = 0;

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
int alphaCounts; // Joystick x-axis rotation analog signal
int betaCounts; // Joystick y-axis rotation analog signal
float alpha; // Joystick x-axis rotation angle
float beta; // Joystick x-axis rotation angle

// Values for the setDesStateSerial() function
float serialInputValue = 0.0f; // User input over the serial line
float sineFrequency = 0.0f; // If using sine wave, its frequency in Hz
float sineTime = 0.0f; // Counter used to determine time in the sine functions (seconds)

// A flag for whether or not the sine sweep should be conducted. User input while the program is 
// running sets this. DON'T SET THIS YOURSELF!
bool sweepFlag = 0;

// SD Card settings
String filePrefix = "flight_data";
String fileExtension = ".csv";
String fileName;

File dataFile;

Vector3f desState(0,0,0);
Vector3f currState(0,0,0);
Vector3f pidOutputVals(0,0,0);

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

//========================================================================================================================//
//                                                      VOID SETUP                                                        //                           
//========================================================================================================================//

void setup() {
  Serial.begin(500000); //USB serial
  delay(500);

  //Initialize all pins
  pinMode(13, 1); //Pin 13 LED blinker on board, do not modify 
  pinMode(m1Pin, 1);
  pinMode(m2Pin, 1);
  pinMode(m3Pin, 1);
  pinMode(m4Pin, 1);
  pinMode(m5Pin, 1);
  pinMode(m6Pin, 1);
  servo1.attach(servo1Pin, 1000, 2100); //Pin, min PWM value, max PWM value
  servo2.attach(servo2Pin, 1000, 2100);
  servo3.attach(servo3Pin, 1000, 2100);
  servo4.attach(servo4Pin, 1000, 2100);
  servo5.attach(servo5Pin, 1000, 2100);
  servo6.attach(servo6Pin, 1000, 2100);
  servo7.attach(servo7Pin, 1000, 2100);

 // Attach the iris servo and close it
 iris.attach(irisPin);

  //Set built in LED to turn on to signal startup
  digitalWrite(13, 1);

  delay(5);

  //Initialize SD card
  Serial.print("Initializing SD card...");
  // see if the card is present and can be initialized:
  if (SD.begin(chipSelect)) {
  Serial.println("card initialized.");
  SD_is_present = 1;
  int fileIncrement = 0;
  fileName = filePrefix + String(fileIncrement) + fileExtension;
  // Check whether or not the file name exists
  while(SD.exists(fileName.c_str())) {
   // Increment the filename if it exists and try again
   fileIncrement++;
   fileName = filePrefix + String(fileIncrement) + fileExtension;
  }
  dataFile = SD.open(fileName.c_str(), 1);
    String csvHeader =
   "roll_imu,pitch_imu,yaw_imu,alpha,beta,roll_des,pitch_des,yaw_des,throttle_des,roll_pid,pitch_pid,yaw_pid,radio_ch1,radio_ch2,radio_ch3,radio_ch4,radio_ch5,radio_ch6,radio_ch7,radio_ch8,radio_ch9,radio_ch10,radio_ch11,radio_ch12,radio_ch13,GyroX,GyroY,GyroZ,AccX,AccY,AccZ,s1_command,s2_command,s3_command,s4_command,kp_roll,ki_roll,kd_roll,kp_pitch,ki_pitch,kd_pitch,kp_yaw,ki_yaw,kd_yaw,failsafeTriggered";
  dataFile.println(csvHeader);
  dataFile.close();
  }
 else {
    Serial.println("Card failed, or not present");
  SD_is_present = 0;
 }

  //Initialize radio communication (defined in header file)
  radioSetup();

  //Set radio channels to default (safe) values before entering main loop
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

  //Initialize IMU communication
  IMUinit();

  delay(5);

  //Get IMU error to zero accelerometer and gyro readings, assuming vehicle is level when powered up
  //Calibration parameters printed to serial monitor. Paste these in the user specified variables 
 //section, then comment this out forever.
  //calculate_IMU_error();

  //Arm servo channels
  servo1.write(0); //Command servo angle from 0-180 degrees (1000 to 2000 PWM)
  servo2.write(0); //Set these to 90 for servos if you do not want them to briefly max out on startup
  servo3.write(0); //Keep these at 0 if you are using servo outputs for motors
  servo4.write(0);
  servo5.write(0);
  servo6.write(0);
  servo7.write(0);

  delay(5);

  //PROPS OFF. Uncomment this to calibrate your ESCs by setting throttle stick to max, powering on,
  //and lowering throttle to zero after the beeps
  //calibrateESCs();
  //Code will not proceed past here if this function is uncommented!

  // Calibrate the joystick. Will be in an infinite loop.
 //calibrateJoystick();

  //Arm OneShot125 motors
  m1_command_PWM = 125; //Command OneShot125 ESC from 125 to 250us pulse length
  m2_command_PWM = 125;
  m3_command_PWM = 125;
  m4_command_PWM = 125;
  m5_command_PWM = 125;
  m6_command_PWM = 125;
  armMotors(); //Loop over commandMotors() until ESCs happily arm

  //Indicate entering main loop with 3 quick blinks
  setupBlink(3,160,70); //numBlinks, upTime (ms), downTime (ms)

 //If using MPU9250 IMU, uncomment for one-time magnetometer calibration (may need to repeat for
 //new locations) Generates magentometer error and scale factors to be pasted in user-specified
 //variables section
  //calibrateMagnetometer();
 doneWithSetup = 1;
}



//================================================================================================//
//                                      MAIN LOOP                                                 //                           
//================================================================================================//
void loop() {
  //Keep track of what time it is and how much time has elapsed since the last loop
  prev_time = current_time;
  current_time = micros();
  dt = (current_time - prev_time)/1000000.0;

  loopBlink(); //Indicate we are in main loop with short blink every 1.5 seconds

  // Print data at 100hz (uncomment one at a time for troubleshooting) - SELECT ONE:
 // Prints radio pwm values (expected: 1000 to 2000)
  //printRadioData();     
 // Prints desired vehicle state commanded in either degrees or deg/sec (expected: +/- maxAXIS for
 // roll, pitch, yaw; 0 to 1 for throttle)
  //printDesiredState();  
 // Prints filtered gyro data direct from IMU (expected: ~ -250 to 250, 0 at rest)
  //printGyroData();      
 // Prints filtered accelerometer data direct from IMU (expected: ~ -2 to 2; x,y 0 when level, z 1
 // when level)
  //printAccelData();     
 // Prints filtered magnetometer data direct from IMU (expected: ~ -300 to 300)
  //printMagData();       
 // Prints roll, pitch, and yaw angles in degrees from Madgwick filter (expected: degrees, 0 when
 // level)
  //printRollPitchYaw();  
 // Combines printRollPitchYaw() with printDesiredState() and prints in tab separted values in the
 // order specified in the function.
  //printRollPitchYawAndDesired(); 
 // Prints computed stabilized PID variables from controller and desired setpoint (expected: ~ -1
 // to 1)
  //printPIDoutput();     
 // Prints the values being written to the motors (expected: 120 to 250)
  //printMotorCommands(); 
 // Prints the values being written to the servos (expected: 0 to 180)
  //printServoCommands(); 
 // Prints the time between loops in microseconds (expected: microseconds between loop iterations)
  //printLoopRate();      
 // Prints the angles alpha, beta, pitch, roll, alpha + roll, beta + pitch
 //printRIPAngles();
 // Prints desired and imu roll state for serial plotter
 displayRoll();
 // Prints desired and imu pitch state for serial plotter
 //displayPitch();

 // Check for whether or not the iris should be open
 if (channel_6_pwm < 1500) {
  irisFlag = 0;
  closeIris();
 }
 else {
  irisFlag = 1;
  openIris();
 }
 // Check to see whether a sine sweep is to be conducted
 if (channel_7_pwm > 1750) {
  conductSineSweep = 1;
 }
 else {
  conductSineSweep = 0;
  sineTime = 0;
 }

 //Save to SD card
 if (SD_is_present && current_time - print_counterSD > 10000) {

    print_counterSD = micros();
    String dataString;

    dataString = getDataString();
    dataFile = SD.open(fileName.c_str(), 1);
    if (dataFile) {
      dataFile.println(dataString);
      dataFile.close();
    }
    // if the file isn't open, pop up an error:
    else {
      Serial.println("error opening datalog.txt");
    }
  }


  //Get vehicle state
  getIMUdata(); //Pulls raw gyro, accelerometer, and magnetometer data from IMU and LP filters to remove noise
  Madgwick(GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ, MagY, -MagX, MagZ, dt); //Updates roll_IMU, pitch_IMU, and yaw_IMU angle estimates (degrees)

  //Compute desired state
  getDesState(); //Convert raw commands to normalized values based on saturated control limits


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

  //PID Controller - SELECT ONE:
 //controlANGLE();
  //controlANGLE2(); //Stabilize on angle setpoint using cascaded method. Rate controller must be tuned well first!
  //controlRATE(); //Stabilize on rate setpoint

 getPScale();
 getIScale();
 getDScale();

 // Linear algebra PID function
 desState[0] = pitch_des;
 desState[1] = yaw_des;
 desState[2] = roll_des;
 currState[0] = pitch_IMU;
 currState[1] = yaw_IMU;
 currState[2] = roll_IMU;
 pidOutputVals = pidOutput(desState, currState, (P_gains*P_gainScale), (I_gains*I_gainScale), (D_gains*D_gainScale), dt, channel_1_pwm <
  1060, GyroX, GyroY, GyroZ);
 pitch_PID = pidOutputVals[0];
 yaw_PID = pidOutputVals[1];
 roll_PID = pidOutputVals[2];


  //Actuator mixing and scaling to PWM values
  controlMixer(); //Mixes PID outputs to scaled actuator commands -- custom mixing assignments done here
  scaleCommands(); //Scales motor commands to 125 to 250 range (oneshot125 protocol) and servo PWM commands to 0 to 180 (for servo library)

  //Throttle cut check
  throttleCut(); //Directly sets motor commands to low based on state of ch5

  //Command actuators
  commandMotors(); //Sends command pulses to each motor pin using OneShot125 protocol
  servo1.write(s1_command_PWM); //Writes PWM value to servo object
  servo2.write(s2_command_PWM);
  servo3.write(s3_command_PWM);
  servo4.write(s4_command_PWM);
  servo5.write(s5_command_PWM);
  servo6.write(s6_command_PWM);
  servo7.write(s7_command_PWM);

  //Get vehicle commands for next loop iteration
  getCommands(); //Pulls current available radio commands
  failSafe(); //Prevent failures in event of bad receiver connection, defaults to failsafe values assigned in setup

 // Get the joystick angle from their potentiometers
  getJoyAngle();

  //Regulate loop rate
  loopRate(2000); //Do not exceed 2000Hz, all filter parameters tuned to 2000Hz by default
}



//========================================================================================================================//
//                                                      FUNCTIONS                                                         //                           
//========================================================================================================================//



void controlMixer() {
  //DESCRIPTION: Mixes scaled commands from PID controller to actuator outputs based on vehicle configuration
  /*

   * Takes roll_PID, pitch_PID, and yaw_PID computed from the PID controller and appropriately mixes them for the desired

   * vehicle configuration. For example on a quadcopter, the left two motors should have +roll_PID while the right two motors

   * should have -roll_PID. Front two should have -pitch_PID and the back two should have +pitch_PID etc... every motor has

   * normalized (0 to 1) thro_des command for throttle control. Can also apply direct unstabilized commands from the transmitter with 

   * roll_passthru, pitch_passthru, and yaw_passthu. mX_command_scaled and sX_command scaled variables are used in scaleCommands() 

   * in preparation to be sent to the motor ESCs and servos.

   * 

   *Relevant variables:

   *thro_des - direct thottle control

   *roll_PID, pitch_PID, yaw_PID - stabilized axis variables

   *roll_passthru, pitch_passthru, yaw_passthru - direct unstabilized command passthrough

   *channel_6_pwm - free auxillary channel, can be used to toggle things with an 'if' statement

   */
# 769 "/home/james/Documents/dRehmFlight/Versions/dRehmFlight_quad/dRehmFlight_quad.ino"
  //Quad mixing - EXAMPLE
  m1_command_scaled = thro_des - pitch_PID + roll_PID + yaw_PID; //Front Left
  m2_command_scaled = thro_des - pitch_PID - roll_PID - yaw_PID; //Front Right
  m3_command_scaled = thro_des + pitch_PID - roll_PID + yaw_PID; //Back Right
  m4_command_scaled = thro_des + pitch_PID + roll_PID - yaw_PID; //Back Left
  m5_command_scaled = 0;
  m6_command_scaled = 0;

  //0.5 is centered servo, 0.0 is zero throttle if connecting to ESC for conventional PWM, 1.0 is max throttle
  s1_command_scaled = thro_des - pitch_PID + roll_PID + yaw_PID; //Front Left
  s2_command_scaled = thro_des - pitch_PID - roll_PID - yaw_PID; //Front Right
  s3_command_scaled = thro_des + pitch_PID - roll_PID + yaw_PID; //Back Right
  s4_command_scaled = thro_des + pitch_PID + roll_PID - yaw_PID; //Back Left
  s5_command_scaled = 0;
  s6_command_scaled = 0;
  s7_command_scaled = 0;
}

void IMUinit() {
  //DESCRIPTION: Initialize IMU
  /*

   * Don't worry about how this works.

   */
# 793 "/home/james/Documents/dRehmFlight/Versions/dRehmFlight_quad/dRehmFlight_quad.ino"
    Wire.begin();
    Wire.setClock(1000000); //Note this is 2.5 times the spec sheet 400 kHz max...

    mpu6050.initialize();

    if (mpu6050.testConnection() == false) {
      Serial.println("MPU6050 initialization unsuccessful");
      Serial.println("Check MPU6050 wiring or try cycling power");
      while(1) {}
    }

    //From the reset state all registers should be 0x00, so we should be at
    //max sample rate with digital low pass filter(s) off.  All we need to
    //do is set the desired fullscale ranges
    mpu6050.setFullScaleGyroRange(0x00);
    mpu6050.setFullScaleAccelRange(0x00);
# 831 "/home/james/Documents/dRehmFlight/Versions/dRehmFlight_quad/dRehmFlight_quad.ino"
}

void getIMUdata() {
  //DESCRIPTION: Request full dataset from IMU and LP filter gyro, accelerometer, and magnetometer data
  /*

   * Reads accelerometer, gyro, and magnetometer data from IMU as AccX, AccY, AccZ, GyroX, GyroY, GyroZ, MagX, MagY, MagZ. 

   * These values are scaled according to the IMU datasheet to put them into correct units of g's, deg/sec, and uT. A simple first-order

   * low-pass filter is used to get rid of high frequency noise in these raw signals. Generally you want to cut

   * off everything past 80Hz, but if your loop rate is not fast enough, the low pass filter will cause a lag in

   * the readings. The filter parameters B_gyro and B_accel are set to be good for a 2kHz loop rate. Finally,

   * the constant errors found in calculate_IMU_error() on startup are subtracted from the accelerometer and gyro readings.

   */
# 843 "/home/james/Documents/dRehmFlight/Versions/dRehmFlight_quad/dRehmFlight_quad.ino"
  int16_t AcX,AcY,AcZ,GyX,GyY,GyZ,MgX,MgY,MgZ;


    mpu6050.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);




 //Accelerometer
  AccX = AcX / 16384.0; //G's
  AccY = AcY / 16384.0;
  AccZ = AcZ / 16384.0;
  //Correct the outputs with the calculated error values
  AccX = AccX - AccErrorX;
  AccY = AccY - AccErrorY;
  AccZ = AccZ - AccErrorZ;
  //LP filter accelerometer data
  AccX = (1.0 - B_accel)*AccX_prev + B_accel*AccX;
  AccY = (1.0 - B_accel)*AccY_prev + B_accel*AccY;
  AccZ = (1.0 - B_accel)*AccZ_prev + B_accel*AccZ;
  AccX_prev = AccX;
  AccY_prev = AccY;
  AccZ_prev = AccZ;

  //Gyro
  GyroX = GyX / 131.0; //deg/sec
  GyroY = GyY / 131.0;
  GyroZ = GyZ / 131.0;
  //Correct the outputs with the calculated error values
  GyroX = GyroX - GyroErrorX;
  GyroY = GyroY - GyroErrorY;
  GyroZ = GyroZ - GyroErrorZ;
  //LP filter gyro data
  GyroX = (1.0 - B_gyro)*GyroX_prev + B_gyro*GyroX;
  GyroY = (1.0 - B_gyro)*GyroY_prev + B_gyro*GyroY;
  GyroZ = (1.0 - B_gyro)*GyroZ_prev + B_gyro*GyroZ;
  GyroX_prev = GyroX;
  GyroY_prev = GyroY;
  GyroZ_prev = GyroZ;

  //Magnetometer
  MagX = MgX/6.0; //uT
  MagY = MgY/6.0;
  MagZ = MgZ/6.0;
  //Correct the outputs with the calculated error values
  MagX = (MagX - MagErrorX)*MagScaleX;
  MagY = (MagY - MagErrorY)*MagScaleY;
  MagZ = (MagZ - MagErrorZ)*MagScaleZ;
  //LP filter magnetometer data
  MagX = (1.0 - B_mag)*MagX_prev + B_mag*MagX;
  MagY = (1.0 - B_mag)*MagY_prev + B_mag*MagY;
  MagZ = (1.0 - B_mag)*MagZ_prev + B_mag*MagZ;
  MagX_prev = MagX;
  MagY_prev = MagY;
  MagZ_prev = MagZ;
}

void calculate_IMU_error() {
  //DESCRIPTION: Computes IMU accelerometer and gyro error on startup. Note: vehicle should be powered up on flat surface
  /*

   * Don't worry too much about what this is doing. The error values it computes are applied to the raw gyro and 

   * accelerometer values AccX, AccY, AccZ, GyroX, GyroY, GyroZ in getIMUdata(). This eliminates drift in the

   * measurement. 

   */
# 907 "/home/james/Documents/dRehmFlight/Versions/dRehmFlight_quad/dRehmFlight_quad.ino"
  int16_t AcX,AcY,AcZ,GyX,GyY,GyZ,MgX,MgY,MgZ;
  AccErrorX = 0.0;
  AccErrorY = 0.0;
  AccErrorZ = 0.0;
  GyroErrorX = 0.0;
  GyroErrorY= 0.0;
  GyroErrorZ = 0.0;

  //Read IMU values 12000 times
  int c = 0;
  while (c < 12000) {

      mpu6050.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);




    AccX = AcX / 16384.0;
    AccY = AcY / 16384.0;
    AccZ = AcZ / 16384.0;
    GyroX = GyX / 131.0;
    GyroY = GyY / 131.0;
    GyroZ = GyZ / 131.0;

    //Sum all readings
    AccErrorX = AccErrorX + AccX;
    AccErrorY = AccErrorY + AccY;
    AccErrorZ = AccErrorZ + AccZ;
    GyroErrorX = GyroErrorX + GyroX;
    GyroErrorY = GyroErrorY + GyroY;
    GyroErrorZ = GyroErrorZ + GyroZ;
    c++;
  }
  //Divide the sum by 12000 to get the error value
  AccErrorX = AccErrorX / c;
  AccErrorY = AccErrorY / c;
  AccErrorZ = AccErrorZ / c - 1.0;
  GyroErrorX = GyroErrorX / c;
  GyroErrorY = GyroErrorY / c;
  GyroErrorZ = GyroErrorZ / c;

  Serial.print("float AccErrorX = ");
  Serial.print(AccErrorX);
  Serial.println(";");
  Serial.print("float AccErrorY = ");
  Serial.print(AccErrorY);
  Serial.println(";");
  Serial.print("float AccErrorZ = ");
  Serial.print(AccErrorZ);
  Serial.println(";");

  Serial.print("float GyroErrorX = ");
  Serial.print(GyroErrorX);
  Serial.println(";");
  Serial.print("float GyroErrorY = ");
  Serial.print(GyroErrorY);
  Serial.println(";");
  Serial.print("float GyroErrorZ = ");
  Serial.print(GyroErrorZ);
  Serial.println(";");

  Serial.println("Paste these values in user specified variables section and comment out calculate_IMU_error() in void setup.");
}

void calibrateAttitude() {
  //DESCRIPTION: Used to warm up the main loop to allow the madwick filter to converge before commands can be sent to the actuators
  //Assuming vehicle is powered up on level surface!
  /*

   * This function is used on startup to warm up the attitude estimation and is what causes startup to take a few seconds

   * to boot. 

   */
# 978 "/home/james/Documents/dRehmFlight/Versions/dRehmFlight_quad/dRehmFlight_quad.ino"
  //Warm up IMU and madgwick filter in simulated main loop
  for (int i = 0; i <= 10000; i++) {
    prev_time = current_time;
    current_time = micros();
    dt = (current_time - prev_time)/1000000.0;
    getIMUdata();
    Madgwick(GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ, MagY, -MagX, MagZ, dt);
    loopRate(2000); //do not exceed 2000Hz
  }
}

void Madgwick(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float invSampleFreq) {
  //DESCRIPTION: Attitude estimation through sensor fusion - 9DOF
  /*

   * This function fuses the accelerometer gyro, and magnetometer readings AccX, AccY, AccZ, GyroX, GyroY, GyroZ, MagX, MagY, and MagZ for attitude estimation.

   * Don't worry about the math. There is a tunable parameter B_madgwick in the user specified variable section which basically

   * adjusts the weight of gyro data in the state estimate. Higher beta leads to noisier estimate, lower 

   * beta leads to slower to respond estimate. It is currently tuned for 2kHz loop rate. This function updates the roll_IMU,

   * pitch_IMU, and yaw_IMU variables which are in degrees. If magnetometer data is not available, this function calls Madgwick6DOF() instead.

   */
# 998 "/home/james/Documents/dRehmFlight/Versions/dRehmFlight_quad/dRehmFlight_quad.ino"
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float hx, hy;
  float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

  //use 6DOF algorithm if MPU6050 is being used

    Madgwick6DOF(gx, gy, gz, ax, ay, az, invSampleFreq);
    return;


  //Use 6DOF algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
  if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
    Madgwick6DOF(gx, gy, gz, ax, ay, az, invSampleFreq);
    return;
  }

  //Convert gyroscope degrees/sec to radians/sec
  gx *= 0.0174533f;
  gy *= 0.0174533f;
  gz *= 0.0174533f;

  //Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  //Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    //Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    //Normalise magnetometer measurement
    recipNorm = invSqrt(mx * mx + my * my + mz * mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    //Auxiliary variables to avoid repeated arithmetic
    _2q0mx = 2.0f * q0 * mx;
    _2q0my = 2.0f * q0 * my;
    _2q0mz = 2.0f * q0 * mz;
    _2q1mx = 2.0f * q1 * mx;
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _2q0q2 = 2.0f * q0 * q2;
    _2q2q3 = 2.0f * q2 * q3;
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;

    //Reference direction of Earth's magnetic field
    hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
    hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
    _2bx = sqrtf(hx * hx + hy * hy);
    _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    //Gradient decent algorithm corrective step
    s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    //Apply feedback step
    qDot1 -= B_madgwick * s0;
    qDot2 -= B_madgwick * s1;
    qDot3 -= B_madgwick * s2;
    qDot4 -= B_madgwick * s3;
  }

  //Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * invSampleFreq;
  q1 += qDot2 * invSampleFreq;
  q2 += qDot3 * invSampleFreq;
  q3 += qDot4 * invSampleFreq;

  //Normalize quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;

  //compute angles - NWU
  roll_IMU = atan2(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2)*57.29577951; //degrees
  pitch_IMU = -asin(-2.0f * (q1*q3 - q0*q2))*57.29577951; //degrees
  yaw_IMU = -atan2(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3)*57.29577951; //degrees
}

void Madgwick6DOF(float gx, float gy, float gz, float ax, float ay, float az, float invSampleFreq) {
  //DESCRIPTION: Attitude estimation through sensor fusion - 6DOF
  /*

   * See description of Madgwick() for more information. This is a 6DOF implimentation for when magnetometer data is not

   * available (for example when using the recommended MPU6050 IMU for the default setup).

   */
# 1115 "/home/james/Documents/dRehmFlight/Versions/dRehmFlight_quad/dRehmFlight_quad.ino"
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  //Convert gyroscope degrees/sec to radians/sec
  gx *= 0.0174533f;
  gy *= 0.0174533f;
  gz *= 0.0174533f;

  //Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  //Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    //Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    //Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    //Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); //normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    //Apply feedback step
    qDot1 -= B_madgwick * s0;
    qDot2 -= B_madgwick * s1;
    qDot3 -= B_madgwick * s2;
    qDot4 -= B_madgwick * s3;
  }

  //Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * invSampleFreq;
  q1 += qDot2 * invSampleFreq;
  q2 += qDot3 * invSampleFreq;
  q3 += qDot4 * invSampleFreq;

  //Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;

  //Compute angles
  roll_IMU = atan2(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2)*57.29577951; //degrees
  pitch_IMU = -asin(-2.0f * (q1*q3 - q0*q2))*57.29577951; //degrees
  yaw_IMU = -atan2(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3)*57.29577951; //degrees
}

void setDesStateSerial(int controlledAxis) {
 //DESCRIPTION: Sets the desired pitch and roll angles based on user input over USB
 /*

		Input:

			var 							type		descr

			===     					====		=====

			controlledAxis 		int 		The axis about which the user's serial inputs set the desired angle.

																1: roll

																2: pitch

	*/
# 1201 "/home/james/Documents/dRehmFlight/Versions/dRehmFlight_quad/dRehmFlight_quad.ino"
 if (Serial.available()) {
  serialInputValue = Serial.parseFloat();
  while (Serial.available() !=0) {
   Serial.read();
  }
 }

 float desiredAngle = 0;

 if (useSineWave) {
  sineFrequency = static_cast<float>(serialInputValue);
  desiredAngle = 10*sin(2*3.1415926535897932384626433832795*sineFrequency*sineTime); // Set the output to be a sin wave between -5 and 5 degrees
  sineTime = sineTime + 1/2000.0f;
 }
 else {
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
 //DESCRIPTION: Performs a sine sweep from minFreq (Hz) to maxFreq (Hz) over sweepTime (seconds)
 /*

		Input:

			var 							type		descr

			===     					====		=====

			controlledAxis 		int 		The axis about which the user's serial inputs set the desired angle.

																1: roll

																2: pitch

			minFreq 							float 	The starting frequency for the sine sweep in Hz

			maxFreq 							float   The ending frequency for the sine sweep in Hz

			sweepTime 				float 	The time period to perform the sweep over

	*/
# 1244 "/home/james/Documents/dRehmFlight/Versions/dRehmFlight_quad/dRehmFlight_quad.ino"
 float desiredAngle = 0;
  float amp = 10; // Sine wave amplitude in degrees
  //if (Serial.available()) {
  //  sweepFlag = 1;
  //  while (Serial.available() !=0) {
  //      Serial.read();
  //    }
 //}
  //if (sweepFlag){
    desiredAngle = amp*sin(3.1415926535897932384626433832795*(maxFreq - minFreq)/pow(sweepTime, 2)*pow(sineTime, 3) + 2*3.1415926535897932384626433832795*minFreq*sineTime);
    if (sineTime > sweepTime) {
      desiredAngle = 0;
    }
    sineTime = sineTime + 1/2000.0f;
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
 if (channel_9_pwm < 1250){
  desiredAngle = 15.0f;
 }
 else if (channel_9_pwm > 1750) {
  desiredAngle = -15.0f;
 }
 else {
  desiredAngle = 0.0f;
 }
 roll_des = desiredAngle;
}
void pitchStep() {
 float desiredAngle;
 if (channel_9_pwm < 1250){
  desiredAngle = 15.0f;
 }
 else if (channel_9_pwm > 1750) {
  desiredAngle = -15.0f;
 }
 else {
  desiredAngle = 0.0f;
 }
 pitch_des = desiredAngle;
}

void getDesState() {
  //DESCRIPTION: Normalizes desired control values to appropriate values
  /*

   * Updates the desired state variables thro_des, roll_des, pitch_des, and yaw_des. These are computed by using the raw

   * RC pwm commands and scaling them to be within our limits defined in setup. thro_des stays within 0 to 1 range.

   * roll_des and pitch_des are scaled to be within max roll/pitch amount in either degrees (angle mode) or degrees/sec

   * (rate mode). yaw_des is scaled to be within max yaw in degrees/sec. Also creates roll_passthru, pitch_passthru, and

   * yaw_passthru variables, to be used in commanding motors/servos with direct unstabilized commands in controlMixer().

   */
# 1308 "/home/james/Documents/dRehmFlight/Versions/dRehmFlight_quad/dRehmFlight_quad.ino"
  thro_des = (channel_1_pwm - 1000.0)/1000.0; //Between 0 and 1
  roll_des = (channel_2_pwm - 1500.0)/500.0; //Between -1 and 1
  pitch_des = (channel_3_pwm - 1500.0)/500.0; //Between -1 and 1
  yaw_des = (channel_4_pwm - 1500.0)/500.0; //Between -1 and 1
  roll_passthru = roll_des/2.0; //Between -0.5 and 0.5
  pitch_passthru = pitch_des/2.0; //Between -0.5 and 0.5
  yaw_passthru = yaw_des/2.0; //Between -0.5 and 0.5
  // Desired pendulum angle
 // OLD version:
 //alpha_des = -roll_IMU/alpha_max; // Between -1 and 1
 //beta_des = -pitch_IMU/beta_max; // Between -1 and 1
 alpha_des = roll_des;
 beta_des = pitch_des;

  //Constrain within normalized bounds
  thro_des = ({ typeof(thro_des) _amt = (thro_des); typeof(0.0) _low = (0.0); typeof(1.0) _high = (1.0); (_amt < _low) ? _low : ((_amt > _high) ? _high : _amt); }); //Between 0 and 1
  roll_des = ({ typeof(roll_des) _amt = (roll_des); typeof(-1.0) _low = (-1.0); typeof(1.0) _high = (1.0); (_amt < _low) ? _low : ((_amt > _high) ? _high : _amt); })*maxRoll; //Between -maxRoll and +maxRoll
  pitch_des = ({ typeof(pitch_des) _amt = (pitch_des); typeof(-1.0) _low = (-1.0); typeof(1.0) _high = (1.0); (_amt < _low) ? _low : ((_amt > _high) ? _high : _amt); })*maxPitch; //Between -maxPitch and +maxPitch
  yaw_des = ({ typeof(yaw_des) _amt = (yaw_des); typeof(-1.0) _low = (-1.0); typeof(1.0) _high = (1.0); (_amt < _low) ? _low : ((_amt > _high) ? _high : _amt); })*maxYaw; //Between -maxYaw and +maxYaw
 alpha_des = ({ typeof(alpha_des) _amt = (alpha_des); typeof(-1.0) _low = (-1.0); typeof(1.0) _high = (1.0); (_amt < _low) ? _low : ((_amt > _high) ? _high : _amt); })*alpha_max;
 beta_des = ({ typeof(beta_des) _amt = (beta_des); typeof(-1.0) _low = (-1.0); typeof(1.0) _high = (1.0); (_amt < _low) ? _low : ((_amt > _high) ? _high : _amt); })*beta_max;
  roll_passthru = ({ typeof(roll_passthru) _amt = (roll_passthru); typeof(-0.5) _low = (-0.5); typeof(0.5) _high = (0.5); (_amt < _low) ? _low : ((_amt > _high) ? _high : _amt); });
  pitch_passthru = ({ typeof(pitch_passthru) _amt = (pitch_passthru); typeof(-0.5) _low = (-0.5); typeof(0.5) _high = (0.5); (_amt < _low) ? _low : ((_amt > _high) ? _high : _amt); });
  yaw_passthru = ({ typeof(yaw_passthru) _amt = (yaw_passthru); typeof(-0.5) _low = (-0.5); typeof(0.5) _high = (0.5); (_amt < _low) ? _low : ((_amt > _high) ? _high : _amt); });
}


void controlANGLE() {
  //DESCRIPTION: Computes control commands based on state error (angle)
  /*

   * Basic PID control to stablize on angle setpoint based on desired states roll_des, pitch_des, and yaw_des computed in 

   * getDesState(). Error is simply the desired state minus the actual state (ex. roll_des - roll_IMU). Two safety features

   * are implimented here regarding the I terms. The I terms are saturated within specified limits on startup to prevent 

   * excessive buildup. This can be seen by holding the vehicle at an angle and seeing the motors ramp up on one side until

   * they've maxed out throttle...saturating I to a specified limit fixes this. The second feature defaults the I terms to 0

   * if the throttle is at the minimum setting. This means the motors will not start spooling up on the ground, and the I 

   * terms will always start from 0 on takeoff. This function updates the variables roll_PID, pitch_PID, and yaw_PID which

   * can be thought of as 1-D stablized signals. They are mixed to the configuration of the vehicle in controlMixer().

   */
# 1348 "/home/james/Documents/dRehmFlight/Versions/dRehmFlight_quad/dRehmFlight_quad.ino"
  //Roll
  error_roll = roll_des - roll_IMU;
  integral_roll = integral_roll_prev + error_roll*dt;
  if (channel_1_pwm < 1060) { //Don't let integrator build if throttle is too low
    integral_roll = 0;
  }
  integral_roll = ({ typeof(integral_roll) _amt = (integral_roll); typeof(-i_limit) _low = (-i_limit); typeof(i_limit) _high = (i_limit); (_amt < _low) ? _low : ((_amt > _high) ? _high : _amt); }); //Saturate integrator to prevent unsafe buildup
  derivative_roll = GyroX;
  roll_PID = 0.01*(Kp_roll_angle*error_roll + Ki_roll_angle*integral_roll - Kd_roll_angle*derivative_roll); //Scaled by .01 to bring within -1 to 1 range

  //Pitch
  error_pitch = pitch_des - pitch_IMU;
  integral_pitch = integral_pitch_prev + error_pitch*dt;
  if (channel_1_pwm < 1060) { //Don't let integrator build if throttle is too low
    integral_pitch = 0;
  }
  integral_pitch = ({ typeof(integral_pitch) _amt = (integral_pitch); typeof(-i_limit) _low = (-i_limit); typeof(i_limit) _high = (i_limit); (_amt < _low) ? _low : ((_amt > _high) ? _high : _amt); }); //Saturate integrator to prevent unsafe buildup
  derivative_pitch = GyroY;
  pitch_PID = .01*(Kp_pitch_angle*error_pitch + Ki_pitch_angle*integral_pitch - Kd_pitch_angle*derivative_pitch); //Scaled by .01 to bring within -1 to 1 range

  //Yaw, stablize on rate from GyroZ
  error_yaw = yaw_des - GyroZ;
  integral_yaw = integral_yaw_prev + error_yaw*dt;
  if (channel_1_pwm < 1060) { //Don't let integrator build if throttle is too low
    integral_yaw = 0;
  }
  integral_yaw = ({ typeof(integral_yaw) _amt = (integral_yaw); typeof(-i_limit) _low = (-i_limit); typeof(i_limit) _high = (i_limit); (_amt < _low) ? _low : ((_amt > _high) ? _high : _amt); }); //Saturate integrator to prevent unsafe buildup
  derivative_yaw = (error_yaw - error_yaw_prev)/dt;
  yaw_PID = .01*(Kp_yaw*error_yaw + Ki_yaw*integral_yaw + Kd_yaw*derivative_yaw); //Scaled by .01 to bring within -1 to 1 range

  //Update roll variables
  integral_roll_prev = integral_roll;
  //Update pitch variables
  integral_pitch_prev = integral_pitch;
  //Update yaw variables
  error_yaw_prev = error_yaw;
  integral_yaw_prev = integral_yaw;

}


void controlANGLE2() {
  //DESCRIPTION: Computes control commands based on state error (angle) in cascaded scheme
  /*

   * Gives better performance than controlANGLE() but requires much more tuning. Not reccommended for first-time setup.

   * See the documentation for tuning this controller.

   */
# 1395 "/home/james/Documents/dRehmFlight/Versions/dRehmFlight_quad/dRehmFlight_quad.ino"
  //Outer loop - PID on angle
  float roll_des_ol, pitch_des_ol;
  //Roll
  error_roll = roll_des - roll_IMU;
  integral_roll_ol = integral_roll_prev_ol + error_roll*dt;
  if (channel_1_pwm < 1060) { //Don't let integrator build if throttle is too low
    integral_roll_ol = 0;
  }
  integral_roll_ol = ({ typeof(integral_roll_ol) _amt = (integral_roll_ol); typeof(-i_limit) _low = (-i_limit); typeof(i_limit) _high = (i_limit); (_amt < _low) ? _low : ((_amt > _high) ? _high : _amt); }); //Saturate integrator to prevent unsafe buildup
  derivative_roll = (roll_IMU - roll_IMU_prev)/dt;
  roll_des_ol = Kp_roll_angle*error_roll + Ki_roll_angle*integral_roll_ol;// - Kd_roll_angle*derivative_roll;

  //Pitch
  error_pitch = pitch_des - pitch_IMU;
  integral_pitch_ol = integral_pitch_prev_ol + error_pitch*dt;
  if (channel_1_pwm < 1060) { //Don't let integrator build if throttle is too low
    integral_pitch_ol = 0;
  }
  integral_pitch_ol = ({ typeof(integral_pitch_ol) _amt = (integral_pitch_ol); typeof(-i_limit) _low = (-i_limit); typeof(i_limit) _high = (i_limit); (_amt < _low) ? _low : ((_amt > _high) ? _high : _amt); }); //saturate integrator to prevent unsafe buildup
  derivative_pitch = (pitch_IMU - pitch_IMU_prev)/dt;
  pitch_des_ol = Kp_pitch_angle*error_pitch + Ki_pitch_angle*integral_pitch_ol;// - Kd_pitch_angle*derivative_pitch;

  //Apply loop gain, constrain, and LP filter for artificial damping
  float Kl = 30.0;
  roll_des_ol = Kl*roll_des_ol;
  pitch_des_ol = Kl*pitch_des_ol;
  roll_des_ol = ({ typeof(roll_des_ol) _amt = (roll_des_ol); typeof(-240.0) _low = (-240.0); typeof(240.0) _high = (240.0); (_amt < _low) ? _low : ((_amt > _high) ? _high : _amt); });
  pitch_des_ol = ({ typeof(pitch_des_ol) _amt = (pitch_des_ol); typeof(-240.0) _low = (-240.0); typeof(240.0) _high = (240.0); (_amt < _low) ? _low : ((_amt > _high) ? _high : _amt); });
  roll_des_ol = (1.0 - B_loop_roll)*roll_des_prev + B_loop_roll*roll_des_ol;
  pitch_des_ol = (1.0 - B_loop_pitch)*pitch_des_prev + B_loop_pitch*pitch_des_ol;

  //Inner loop - PID on rate
  //Roll
  error_roll = roll_des_ol - GyroX;
  integral_roll_il = integral_roll_prev_il + error_roll*dt;
  if (channel_1_pwm < 1060) { //Don't let integrator build if throttle is too low
    integral_roll_il = 0;
  }
  integral_roll_il = ({ typeof(integral_roll_il) _amt = (integral_roll_il); typeof(-i_limit) _low = (-i_limit); typeof(i_limit) _high = (i_limit); (_amt < _low) ? _low : ((_amt > _high) ? _high : _amt); }); //Saturate integrator to prevent unsafe buildup
  derivative_roll = (error_roll - error_roll_prev)/dt;
  roll_PID = .01*(Kp_roll_rate*error_roll + Ki_roll_rate*integral_roll_il + Kd_roll_rate*derivative_roll); //Scaled by .01 to bring within -1 to 1 range

  //Pitch
  error_pitch = pitch_des_ol - GyroY;
  integral_pitch_il = integral_pitch_prev_il + error_pitch*dt;
  if (channel_1_pwm < 1060) { //Don't let integrator build if throttle is too low
    integral_pitch_il = 0;
  }
  integral_pitch_il = ({ typeof(integral_pitch_il) _amt = (integral_pitch_il); typeof(-i_limit) _low = (-i_limit); typeof(i_limit) _high = (i_limit); (_amt < _low) ? _low : ((_amt > _high) ? _high : _amt); }); //Saturate integrator to prevent unsafe buildup
  derivative_pitch = (error_pitch - error_pitch_prev)/dt;
  pitch_PID = .01*(Kp_pitch_rate*error_pitch + Ki_pitch_rate*integral_pitch_il + Kd_pitch_rate*derivative_pitch); //Scaled by .01 to bring within -1 to 1 range

  //Yaw
  error_yaw = yaw_des - GyroZ;
  integral_yaw = integral_yaw_prev + error_yaw*dt;
  if (channel_1_pwm < 1060) { //Don't let integrator build if throttle is too low
    integral_yaw = 0;
  }
  integral_yaw = ({ typeof(integral_yaw) _amt = (integral_yaw); typeof(-i_limit) _low = (-i_limit); typeof(i_limit) _high = (i_limit); (_amt < _low) ? _low : ((_amt > _high) ? _high : _amt); }); //Saturate integrator to prevent unsafe buildup
  derivative_yaw = (error_yaw - error_yaw_prev)/dt;
  yaw_PID = .01*(Kp_yaw*error_yaw + Ki_yaw*integral_yaw + Kd_yaw*derivative_yaw); //Scaled by .01 to bring within -1 to 1 range

  //Update roll variables
  integral_roll_prev_ol = integral_roll_ol;
  integral_roll_prev_il = integral_roll_il;
  error_roll_prev = error_roll;
  roll_IMU_prev = roll_IMU;
  roll_des_prev = roll_des_ol;
  //Update pitch variables
  integral_pitch_prev_ol = integral_pitch_ol;
  integral_pitch_prev_il = integral_pitch_il;
  error_pitch_prev = error_pitch;
  pitch_IMU_prev = pitch_IMU;
  pitch_des_prev = pitch_des_ol;
  //Update yaw variables
  error_yaw_prev = error_yaw;
  integral_yaw_prev = integral_yaw;

}

void controlRATE() {
  //DESCRIPTION: Computes control commands based on state error (rate)
  /*

   * See explanation for controlANGLE(). Everything is the same here except the error is now the desired rate - raw gyro reading.

   */
# 1480 "/home/james/Documents/dRehmFlight/Versions/dRehmFlight_quad/dRehmFlight_quad.ino"
  //Roll
  error_roll = roll_des - GyroX;
  integral_roll = integral_roll_prev + error_roll*dt;
  if (channel_1_pwm < 1060) { //Don't let integrator build if throttle is too low
    integral_roll = 0;
  }
  integral_roll = ({ typeof(integral_roll) _amt = (integral_roll); typeof(-i_limit) _low = (-i_limit); typeof(i_limit) _high = (i_limit); (_amt < _low) ? _low : ((_amt > _high) ? _high : _amt); }); //Saturate integrator to prevent unsafe buildup
  derivative_roll = (error_roll - error_roll_prev)/dt;
  roll_PID = .01*(Kp_roll_rate*error_roll + Ki_roll_rate*integral_roll + Kd_roll_rate*derivative_roll); //Scaled by .01 to bring within -1 to 1 range

  //Pitch
  error_pitch = pitch_des - GyroY;
  integral_pitch = integral_pitch_prev + error_pitch*dt;
  if (channel_1_pwm < 1060) { //Don't let integrator build if throttle is too low
    integral_pitch = 0;
  }
  integral_pitch = ({ typeof(integral_pitch) _amt = (integral_pitch); typeof(-i_limit) _low = (-i_limit); typeof(i_limit) _high = (i_limit); (_amt < _low) ? _low : ((_amt > _high) ? _high : _amt); }); //Saturate integrator to prevent unsafe buildup
  derivative_pitch = (error_pitch - error_pitch_prev)/dt;
  pitch_PID = .01*(Kp_pitch_rate*error_pitch + Ki_pitch_rate*integral_pitch + Kd_pitch_rate*derivative_pitch); //Scaled by .01 to bring within -1 to 1 range

  //Yaw, stablize on rate from GyroZ
  error_yaw = yaw_des - GyroZ;
  integral_yaw = integral_yaw_prev + error_yaw*dt;
  if (channel_1_pwm < 1060) { //Don't let integrator build if throttle is too low
    integral_yaw = 0;
  }
  integral_yaw = ({ typeof(integral_yaw) _amt = (integral_yaw); typeof(-i_limit) _low = (-i_limit); typeof(i_limit) _high = (i_limit); (_amt < _low) ? _low : ((_amt > _high) ? _high : _amt); }); //Saturate integrator to prevent unsafe buildup
  derivative_yaw = (error_yaw - error_yaw_prev)/dt;
  yaw_PID = .01*(Kp_yaw*error_yaw + Ki_yaw*integral_yaw + Kd_yaw*derivative_yaw); //Scaled by .01 to bring within -1 to 1 range

  //Update roll variables
  error_roll_prev = error_roll;
  integral_roll_prev = integral_roll;
  GyroX_prev = GyroX;
  //Update pitch variables
  error_pitch_prev = error_pitch;
  integral_pitch_prev = integral_pitch;
  GyroY_prev = GyroY;
  //Update yaw variables
  error_yaw_prev = error_yaw;
  integral_yaw_prev = integral_yaw;
}

void scaleCommands() {
  //DESCRIPTION: Scale normalized actuator commands to values for ESC/Servo protocol
  /*

   * mX_command_scaled variables from the mixer function are scaled to 125-250us for OneShot125 protocol. sX_command_scaled variables from

   * the mixer function are scaled to 0-180 for the servo library using standard PWM.

   * mX_command_PWM are updated here which are used to command the motors in commandMotors(). sX_command_PWM are updated 

   * which are used to command the servos.

   */
# 1531 "/home/james/Documents/dRehmFlight/Versions/dRehmFlight_quad/dRehmFlight_quad.ino"
  //Scaled to 125us - 250us for oneshot125 protocol
  m1_command_PWM = m1_command_scaled*125 + 125;
  m2_command_PWM = m2_command_scaled*125 + 125;
  m3_command_PWM = m3_command_scaled*125 + 125;
  m4_command_PWM = m4_command_scaled*125 + 125;
  m5_command_PWM = m5_command_scaled*125 + 125;
  m6_command_PWM = m6_command_scaled*125 + 125;
  //Constrain commands to motors within oneshot125 bounds
  m1_command_PWM = ({ typeof(m1_command_PWM) _amt = (m1_command_PWM); typeof(125) _low = (125); typeof(250) _high = (250); (_amt < _low) ? _low : ((_amt > _high) ? _high : _amt); });
  m2_command_PWM = ({ typeof(m2_command_PWM) _amt = (m2_command_PWM); typeof(125) _low = (125); typeof(250) _high = (250); (_amt < _low) ? _low : ((_amt > _high) ? _high : _amt); });
  m3_command_PWM = ({ typeof(m3_command_PWM) _amt = (m3_command_PWM); typeof(125) _low = (125); typeof(250) _high = (250); (_amt < _low) ? _low : ((_amt > _high) ? _high : _amt); });
  m4_command_PWM = ({ typeof(m4_command_PWM) _amt = (m4_command_PWM); typeof(125) _low = (125); typeof(250) _high = (250); (_amt < _low) ? _low : ((_amt > _high) ? _high : _amt); });
  m5_command_PWM = ({ typeof(m5_command_PWM) _amt = (m5_command_PWM); typeof(125) _low = (125); typeof(250) _high = (250); (_amt < _low) ? _low : ((_amt > _high) ? _high : _amt); });
  m6_command_PWM = ({ typeof(m6_command_PWM) _amt = (m6_command_PWM); typeof(125) _low = (125); typeof(250) _high = (250); (_amt < _low) ? _low : ((_amt > _high) ? _high : _amt); });

  //Scaled to 0-180 for servo library
  s1_command_PWM = s1_command_scaled*180;
  s2_command_PWM = s2_command_scaled*180;
  s3_command_PWM = s3_command_scaled*180;
  s4_command_PWM = s4_command_scaled*180;
  s5_command_PWM = s5_command_scaled*180;
  s6_command_PWM = s6_command_scaled*180;
  s7_command_PWM = s7_command_scaled*180;
  //Constrain commands to servos within servo library bounds
  s1_command_PWM = ({ typeof(s1_command_PWM) _amt = (s1_command_PWM); typeof(0) _low = (0); typeof(180) _high = (180); (_amt < _low) ? _low : ((_amt > _high) ? _high : _amt); });
  s2_command_PWM = ({ typeof(s2_command_PWM) _amt = (s2_command_PWM); typeof(0) _low = (0); typeof(180) _high = (180); (_amt < _low) ? _low : ((_amt > _high) ? _high : _amt); });
  s3_command_PWM = ({ typeof(s3_command_PWM) _amt = (s3_command_PWM); typeof(0) _low = (0); typeof(180) _high = (180); (_amt < _low) ? _low : ((_amt > _high) ? _high : _amt); });
  s4_command_PWM = ({ typeof(s4_command_PWM) _amt = (s4_command_PWM); typeof(0) _low = (0); typeof(180) _high = (180); (_amt < _low) ? _low : ((_amt > _high) ? _high : _amt); });
  s5_command_PWM = ({ typeof(s5_command_PWM) _amt = (s5_command_PWM); typeof(0) _low = (0); typeof(180) _high = (180); (_amt < _low) ? _low : ((_amt > _high) ? _high : _amt); });
  s6_command_PWM = ({ typeof(s6_command_PWM) _amt = (s6_command_PWM); typeof(0) _low = (0); typeof(180) _high = (180); (_amt < _low) ? _low : ((_amt > _high) ? _high : _amt); });
  s7_command_PWM = ({ typeof(s7_command_PWM) _amt = (s7_command_PWM); typeof(0) _low = (0); typeof(180) _high = (180); (_amt < _low) ? _low : ((_amt > _high) ? _high : _amt); });

}

void getCommands() {
  //DESCRIPTION: Get raw PWM values for every channel from the radio
  /*

   * Updates radio PWM commands in loop based on current available commands. channel_x_pwm is the raw command used in the rest of 

   * the loop. If using a PWM or PPM receiver, the radio commands are retrieved from a function in the readPWM file separate from this one which 

   * is running a bunch of interrupts to continuously update the radio readings. If using an SBUS receiver, the alues are pulled from the SBUS library directly.

   * The raw radio commands are filtered with a first order low-pass filter to eliminate any really high frequency noise. 

   */
# 1574 "/home/james/Documents/dRehmFlight/Versions/dRehmFlight_quad/dRehmFlight_quad.ino"
  //#if defined USE_PPM_RX || defined USE_PWM_RX
  //  channel_1_pwm = getRadioPWM(1);
  //  channel_2_pwm = getRadioPWM(2);
  //  channel_3_pwm = getRadioPWM(3);
  //  channel_4_pwm = getRadioPWM(4);
  //  channel_5_pwm = getRadioPWM(5);
  //  channel_6_pwm = getRadioPWM(6);


    if (sbus.read(&sbusChannels[0], &sbusFailSafe, &sbusLostFrame))
    {
      //sBus scaling below is for Taranis-Plus and X4R-SB
      float scale = 0.615;
      float bias = 895.0;
      channel_1_pwm_pre = sbusChannels[0] * scale + bias;
      channel_2_pwm_pre = sbusChannels[1] * scale + bias;
      channel_3_pwm_pre = sbusChannels[2] * scale + bias;
      channel_4_pwm_pre = sbusChannels[3] * scale + bias;
      channel_5_pwm = sbusChannels[4] * scale + bias;
      channel_6_pwm = sbusChannels[5] * scale + bias;
   channel_7_pwm = sbusChannels[6] * scale + bias;
   channel_8_pwm = sbusChannels[7] * scale + bias;
   channel_9_pwm = sbusChannels[8] * scale + bias;
   channel_10_pwm = sbusChannels[9] * scale + bias;
   channel_11_pwm = sbusChannels[10] * scale + bias;
   channel_12_pwm = sbusChannels[11] * scale + bias;
   channel_13_pwm = sbusChannels[12] * scale + bias;
    }

  //#elif defined USE_DSM_RX
  //  if (DSM.timedOut(micros())) {
  //      //Serial.println("*** DSM RX TIMED OUT ***");
  //  }
  //  else if (DSM.gotNewFrame()) {
  //      uint16_t values[num_DSM_channels];
  //      DSM.getChannelValues(values, num_DSM_channels);

  //      channel_1_pwm = values[0];
  //      channel_2_pwm = values[1];
  //      channel_3_pwm = values[2];
  //      channel_4_pwm = values[3];
  //      channel_5_pwm = values[4];
  //      channel_6_pwm = values[5];
  //  }


  //Low-pass the critical commands and update previous values
  float b = 0.7; //Lower=slower, higher=noiser
  channel_1_pwm_pre = (1.0 - b)*channel_1_pwm_prev + b*channel_1_pwm_pre;
  channel_2_pwm_pre = (1.0 - b)*channel_2_pwm_prev + b*channel_2_pwm_pre;
  channel_3_pwm_pre = (1.0 - b)*channel_3_pwm_prev + b*channel_3_pwm_pre;
  channel_4_pwm_pre = (1.0 - b)*channel_4_pwm_prev + b*channel_4_pwm_pre;

 // Additional cutoff to deal with occasional spikes in recieved radio commands
 d_ch1 = channel_1_pwm_pre - channel_1_pwm_prev;
 d_ch2 = channel_2_pwm_pre - channel_2_pwm_prev;
 d_ch3 = channel_3_pwm_pre - channel_3_pwm_prev;
 d_ch4 = channel_4_pwm_pre - channel_4_pwm_prev;

 if (abs(d_ch1) > cutoff_val && ch1_CutCounter < 5 && doneWithSetup) {
  channel_1_pwm = channel_1_pwm_prev;
  Serial.println("Radio command spike detected (CH1)");
  ch1_CutCounter++;
 } else {
  channel_1_pwm = channel_1_pwm_pre;
  ch1_CutCounter = 0;
 }
 if (abs(d_ch2) > cutoff_val && ch2_CutCounter < 5 && doneWithSetup) {
  channel_2_pwm = channel_2_pwm_prev;
  Serial.println("Radio command spike detected (CH2)");
  ch2_CutCounter++;
 } else {
  channel_2_pwm = channel_2_pwm_pre;
  ch2_CutCounter = 0;
 }
 if (abs(d_ch3) > cutoff_val && ch3_CutCounter < 5 && doneWithSetup) {
  channel_3_pwm = channel_3_pwm_prev;
  Serial.println("Radio command spike detected (CH3)");
  ch3_CutCounter++;
 } else {
  channel_3_pwm = channel_3_pwm_pre;
  ch3_CutCounter = 0;
 }
 if (abs(d_ch4) > cutoff_val && ch4_CutCounter < 5 && doneWithSetup) {
  channel_4_pwm = channel_4_pwm_prev;
  Serial.println("Radio command spike detected (CH4)");
  ch4_CutCounter++;
 } else {
  channel_4_pwm = channel_4_pwm_pre;
  ch4_CutCounter = 0;
 }

 // Update prev values
  channel_1_pwm_prev = channel_1_pwm;
  channel_2_pwm_prev = channel_2_pwm;
  channel_3_pwm_prev = channel_3_pwm;
  channel_4_pwm_prev = channel_4_pwm;



}

void failSafe() {
  //DESCRIPTION: If radio gives garbage values, set all commands to default values
  /*

   * Radio connection failsafe used to check if the getCommands() function is returning acceptable pwm values. If any of 

   * the commands are lower than 800 or higher than 2200, then we can be certain that there is an issue with the radio

   * connection (most likely hardware related). If any of the channels show this failure, then all of the radio commands 

   * channel_x_pwm are set to default failsafe values specified in the setup. Comment out this function when troubleshooting 

   * your radio connection in case any extreme values are triggering this function to overwrite the printed variables.

   */
# 1685 "/home/james/Documents/dRehmFlight/Versions/dRehmFlight_quad/dRehmFlight_quad.ino"
  int minVal = 800;
  int maxVal = 2200;
  int check1 = 0;
  int check2 = 0;
  int check3 = 0;
  int check4 = 0;
  int check5 = 0;
  int check6 = 0;
 failureFlag = 0;

  //Triggers for failure criteria
  if (channel_1_pwm > maxVal || channel_1_pwm < minVal) check1 = 1;
  if (channel_2_pwm > maxVal || channel_2_pwm < minVal) check2 = 1;
  if (channel_3_pwm > maxVal || channel_3_pwm < minVal) check3 = 1;
  if (channel_4_pwm > maxVal || channel_4_pwm < minVal) check4 = 1;
  if (channel_5_pwm > maxVal || channel_5_pwm < minVal) check5 = 1;
  if (channel_6_pwm > maxVal || channel_6_pwm < minVal) check6 = 1;

  //If any failures, set to default failsafe values
  if ((check1 + check2 + check3 + check4 + check5 + check6) > 0) {
    channel_1_pwm = channel_1_fs;
    channel_2_pwm = channel_2_fs;
    channel_3_pwm = channel_3_fs;
    channel_4_pwm = channel_4_fs;
    channel_5_pwm = channel_5_fs;
    channel_6_pwm = channel_6_fs;
  failureFlag = 1;
  }
}

void commandMotors() {
  //DESCRIPTION: Send pulses to motor pins, oneshot125 protocol
  /*

   * My crude implimentation of OneShot125 protocol which sends 125 - 250us pulses to the ESCs (mXPin). The pulselengths being

   * sent are mX_command_PWM, computed in scaleCommands(). This may be replaced by something more efficient in the future.

   */
# 1721 "/home/james/Documents/dRehmFlight/Versions/dRehmFlight_quad/dRehmFlight_quad.ino"
  int wentLow = 0;
  int pulseStart, timer;
  int flagM1 = 0;
  int flagM2 = 0;
  int flagM3 = 0;
  int flagM4 = 0;
  int flagM5 = 0;
  int flagM6 = 0;

  //Write all motor pins high
  digitalWrite(m1Pin, 1);
  digitalWrite(m2Pin, 1);
  digitalWrite(m3Pin, 1);
  digitalWrite(m4Pin, 1);
  digitalWrite(m5Pin, 1);
  digitalWrite(m6Pin, 1);
  pulseStart = micros();

  //Write each motor pin low as correct pulse length is reached
  while (wentLow < 6 ) { //Keep going until final (6th) pulse is finished, then done
    timer = micros();
    if ((m1_command_PWM <= timer - pulseStart) && (flagM1==0)) {
      digitalWrite(m1Pin, 0);
      wentLow = wentLow + 1;
      flagM1 = 1;
    }
    if ((m2_command_PWM <= timer - pulseStart) && (flagM2==0)) {
      digitalWrite(m2Pin, 0);
      wentLow = wentLow + 1;
      flagM2 = 1;
    }
    if ((m3_command_PWM <= timer - pulseStart) && (flagM3==0)) {
      digitalWrite(m3Pin, 0);
      wentLow = wentLow + 1;
      flagM3 = 1;
    }
    if ((m4_command_PWM <= timer - pulseStart) && (flagM4==0)) {
      digitalWrite(m4Pin, 0);
      wentLow = wentLow + 1;
      flagM4 = 1;
    }
    if ((m5_command_PWM <= timer - pulseStart) && (flagM5==0)) {
      digitalWrite(m5Pin, 0);
      wentLow = wentLow + 1;
      flagM5 = 1;
    }
    if ((m6_command_PWM <= timer - pulseStart) && (flagM6==0)) {
      digitalWrite(m6Pin, 0);
      wentLow = wentLow + 1;
      flagM6 = 1;
    }
  }
}

void armMotors() {
  //DESCRIPTION: Sends many command pulses to the motors, to be used to arm motors in the void setup()
  /*  

   *  Loops over the commandMotors() function 50 times with a delay in between, simulating how the commandMotors()

   *  function is used in the main loop. Ensures motors arm within the void setup() where there are some delays

   *  for other processes that sometimes prevent motors from arming.

   */
# 1782 "/home/james/Documents/dRehmFlight/Versions/dRehmFlight_quad/dRehmFlight_quad.ino"
  for (int i = 0; i <= 50; i++) {
    commandMotors();
    delay(2);
  }
}

void calibrateESCs() {
  //DESCRIPTION: Used in void setup() to allow standard ESC calibration procedure with the radio to take place.
  /*  

   *  Simulates the void loop(), but only for the purpose of providing throttle pass through to the motors, so that you can

   *  power up with throttle at full, let ESCs begin arming sequence, and lower throttle to zero. This function should only be

   *  uncommented when performing an ESC calibration.

   */
# 1795 "/home/james/Documents/dRehmFlight/Versions/dRehmFlight_quad/dRehmFlight_quad.ino"
   while (true) {
      prev_time = current_time;
      current_time = micros();
      dt = (current_time - prev_time)/1000000.0;

      digitalWrite(13, 1); //LED on to indicate we are not in main loop

      getCommands(); //Pulls current available radio commands
      failSafe(); //Prevent failures in event of bad receiver connection, defaults to failsafe values assigned in setup
      getDesState(); //Convert raw commands to normalized values based on saturated control limits
      getIMUdata(); //Pulls raw gyro, accelerometer, and magnetometer data from IMU and LP filters to remove noise
      Madgwick(GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ, MagY, -MagX, MagZ, dt); //Updates roll_IMU, pitch_IMU, and yaw_IMU (degrees)
      getDesState(); //Convert raw commands to normalized values based on saturated control limits

      m1_command_scaled = thro_des;
      m2_command_scaled = thro_des;
      m3_command_scaled = thro_des;
      m4_command_scaled = thro_des;
      m5_command_scaled = thro_des;
      m6_command_scaled = thro_des;
      s1_command_scaled = thro_des;
      s2_command_scaled = thro_des;
      s3_command_scaled = thro_des;
      s4_command_scaled = thro_des;
      s5_command_scaled = thro_des;
      s6_command_scaled = thro_des;
      s7_command_scaled = thro_des;
      scaleCommands(); //Scales motor commands to 125 to 250 range (oneshot125 protocol) and servo PWM commands to 0 to 180 (for servo library)

      //throttleCut(); //Directly sets motor commands to low based on state of ch5

      servo1.write(s1_command_PWM);
      servo2.write(s2_command_PWM);
      servo3.write(s3_command_PWM);
      servo4.write(s4_command_PWM);
      servo5.write(s5_command_PWM);
      servo6.write(s6_command_PWM);
      servo7.write(s7_command_PWM);
      commandMotors(); //Sends command pulses to each motor pin using OneShot125 protocol

      //printRadioData(); //Radio pwm values (expected: 1000 to 2000)

      loopRate(2000); //Do not exceed 2000Hz, all filter parameters tuned to 2000Hz by default
   }
}

float floatFaderLinear(float param, float param_min, float param_max, float fadeTime, int state, int loopFreq){
  //DESCRIPTION: Linearly fades a float type variable between min and max bounds based on desired high or low state and time
  /*  

   *  Takes in a float variable, desired minimum and maximum bounds, fade time, high or low desired state, and the loop frequency 

   *  and linearly interpolates that param variable between the maximum and minimum bounds. This function can be called in controlMixer()

   *  and high/low states can be determined by monitoring the state of an auxillarly radio channel. For example, if channel_6_pwm is being 

   *  monitored to switch between two dynamic configurations (hover and forward flight), this function can be called within the logical 

   *  statements in order to fade controller gains, for example between the two dynamic configurations. The 'state' (1 or 0) can be used

   *  to designate the two final options for that control gain based on the dynamic configuration assignment to the auxillary radio channel.

   *  

   */
# 1852 "/home/james/Documents/dRehmFlight/Versions/dRehmFlight_quad/dRehmFlight_quad.ino"
  float diffParam = (param_max - param_min)/(fadeTime*loopFreq); //Difference to add or subtract from param for each loop iteration for desired fadeTime

  if (state == 1) { //Maximum param bound desired, increase param by diffParam for each loop iteration
    param = param + diffParam;
  }
  else if (state == 0) { //Minimum param bound desired, decrease param by diffParam for each loop iteration
    param = param - diffParam;
  }

  param = ({ typeof(param) _amt = (param); typeof(param_min) _low = (param_min); typeof(param_max) _high = (param_max); (_amt < _low) ? _low : ((_amt > _high) ? _high : _amt); }); //Constrain param within max bounds

  return param;
}

float floatFaderLinear2(float param, float param_des, float param_lower, float param_upper, float fadeTime_up, float fadeTime_down, int loopFreq){
  //DESCRIPTION: Linearly fades a float type variable from its current value to the desired value, up or down
  /*  

   *  Takes in a float variable to be modified, desired new position, upper value, lower value, fade time, and the loop frequency 

   *  and linearly fades that param variable up or down to the desired value. This function can be called in controlMixer()

   *  to fade up or down between flight modes monitored by an auxillary radio channel. For example, if channel_6_pwm is being 

   *  monitored to switch between two dynamic configurations (hover and forward flight), this function can be called within the logical 

   *  statements in order to fade controller gains, for example between the two dynamic configurations. 

   *  

   */
# 1876 "/home/james/Documents/dRehmFlight/Versions/dRehmFlight_quad/dRehmFlight_quad.ino"
  if (param > param_des) { //Need to fade down to get to desired
    float diffParam = (param_upper - param_des)/(fadeTime_down*loopFreq);
    param = param - diffParam;
  }
  else if (param < param_des) { //Need to fade up to get to desired
    float diffParam = (param_des - param_lower)/(fadeTime_up*loopFreq);
    param = param + diffParam;
  }

  param = ({ typeof(param) _amt = (param); typeof(param_lower) _low = (param_lower); typeof(param_upper) _high = (param_upper); (_amt < _low) ? _low : ((_amt > _high) ? _high : _amt); }); //Constrain param within max bounds

  return param;
}

void switchRollYaw(int reverseRoll, int reverseYaw) {
  //DESCRIPTION: Switches roll_des and yaw_des variables for tailsitter-type configurations
  /*

   * Takes in two integers (either 1 or -1) corresponding to the desired reversing of the roll axis and yaw axis, respectively.

   * Reversing of the roll or yaw axis may be needed when switching between the two for some dynamic configurations. Inputs of 1, 1 does not 

   * reverse either of them, while -1, 1 will reverse the output corresponding to the new roll axis. 

   * This function may be replaced in the future by a function that switches the IMU data instead (so that angle can also be estimated with the 

   * IMU tilted 90 degrees from default level).

   */
# 1899 "/home/james/Documents/dRehmFlight/Versions/dRehmFlight_quad/dRehmFlight_quad.ino"
  float switch_holder;

  switch_holder = yaw_des;
  yaw_des = reverseYaw*roll_des;
  roll_des = reverseRoll*switch_holder;
}

void throttleCut() {
  //DESCRIPTION: Directly set actuator outputs to minimum value if triggered
  /*

   * Monitors the state of radio command channel_5_pwm and directly sets the mx_command_PWM values to minimum (120 is

   * minimum for oneshot125 protocol, 0 is minimum for standard PWM servo library used) if channel 5 is high. This is the last function 

   * called before commandMotors() is called so that the last thing checked is if the user is giving permission to command

   * the motors to anything other than minimum value. Safety first. 

   */
# 1914 "/home/james/Documents/dRehmFlight/Versions/dRehmFlight_quad/dRehmFlight_quad.ino"
  if (channel_5_pwm > 1500) {
    // m1_command_PWM = 120;
    // m2_command_PWM = 120;
    // m3_command_PWM = 120;
    // m4_command_PWM = 120;
    // m5_command_PWM = 120;
    // m6_command_PWM = 120;

    //Uncomment if using servo PWM variables to control motor ESCs
    s1_command_PWM = 0;
    s2_command_PWM = 0;
    s3_command_PWM = 0;
    s4_command_PWM = 0;
    s5_command_PWM = 0;
    s6_command_PWM = 0;
    s7_command_PWM = 0;
  }
}

void calibrateMagnetometer() {
# 1976 "/home/james/Documents/dRehmFlight/Versions/dRehmFlight_quad/dRehmFlight_quad.ino"
  Serial.println("Error: MPU9250 not selected. Cannot calibrate non-existent magnetometer.");
  while(1); //Halt code so it won't enter main loop until this function commented out
}

void loopRate(int freq) {
  //DESCRIPTION: Regulate main loop rate to specified frequency in Hz
  /*

   * It's good to operate at a constant loop rate for filters to remain stable and whatnot. Interrupt routines running in the

   * background cause the loop rate to fluctuate. This function basically just waits at the end of every loop iteration until 

   * the correct time has passed since the start of the current loop for the desired loop rate in Hz. 2kHz is a good rate to 

   * be at because the loop nominally will run between 2.8kHz - 4.2kHz. This lets us have a little room to add extra computations

   * and remain above 2kHz, without needing to retune all of our filtering parameters.

   */
# 1989 "/home/james/Documents/dRehmFlight/Versions/dRehmFlight_quad/dRehmFlight_quad.ino"
  float invFreq = 1.0/freq*1000000.0;
  unsigned long checker = micros();

  //Sit in loop until appropriate time has passed
  while (invFreq > (checker - current_time)) {
    checker = micros();
  }
}

void loopBlink() {
  //DESCRIPTION: Blink LED on board to indicate main loop is running
  /*

   * It looks cool.

   */
# 2003 "/home/james/Documents/dRehmFlight/Versions/dRehmFlight_quad/dRehmFlight_quad.ino"
  if (current_time - blink_counter > blink_delay) {
    blink_counter = micros();
    digitalWrite(13, blinkAlternate); //Pin 13 is built in LED

    if (blinkAlternate == 1) {
      blinkAlternate = 0;
      blink_delay = 100000;
      }
    else if (blinkAlternate == 0) {
      blinkAlternate = 1;
      blink_delay = 2000000;
      }
  }
}

void setupBlink(int numBlinks,int upTime, int downTime) {
  //DESCRIPTION: Simple function to make LED on board blink as desired
  for (int j = 1; j<= numBlinks; j++) {
    digitalWrite(13, 0);
    delay(downTime);
    digitalWrite(13, 1);
    delay(upTime);
  }
}

void printRadioData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(((const __FlashStringHelper *)(" CH1: ")));
    Serial.print(channel_1_pwm);
    Serial.print(((const __FlashStringHelper *)(" CH2: ")));
    Serial.print(channel_2_pwm);
    Serial.print(((const __FlashStringHelper *)(" CH3: ")));
    Serial.print(channel_3_pwm);
    Serial.print(((const __FlashStringHelper *)(" CH4: ")));
    Serial.print(channel_4_pwm);
    Serial.print(((const __FlashStringHelper *)(" CH5: ")));
    Serial.print(channel_5_pwm);
    Serial.print(((const __FlashStringHelper *)(" CH6: ")));
    Serial.print(channel_6_pwm);
    Serial.print(((const __FlashStringHelper *)(" CH7: ")));
    Serial.print(channel_7_pwm);
    Serial.print(((const __FlashStringHelper *)(" CH8: ")));
    Serial.print(channel_8_pwm);
    Serial.print(((const __FlashStringHelper *)(" CH9: ")));
    Serial.print(channel_9_pwm);
    Serial.print(((const __FlashStringHelper *)(" CH10: ")));
    Serial.print(channel_10_pwm);
    Serial.print(((const __FlashStringHelper *)(" CH11: ")));
    Serial.print(channel_11_pwm);
    Serial.print(((const __FlashStringHelper *)(" CH12: ")));
    Serial.print(channel_12_pwm);
    Serial.print(((const __FlashStringHelper *)(" CH13: ")));
    Serial.println(channel_13_pwm);
  }
}

void printDesiredState() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(((const __FlashStringHelper *)("thro_des: ")));
    Serial.print(thro_des);
    Serial.print(((const __FlashStringHelper *)(" roll_des: ")));
    Serial.print(roll_des);
    Serial.print(((const __FlashStringHelper *)(" pitch_des: ")));
    Serial.print(pitch_des);
    Serial.print(((const __FlashStringHelper *)(" yaw_des: ")));
    Serial.println(yaw_des);
  }
}

void printGyroData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(((const __FlashStringHelper *)("GyroX: ")));
    Serial.print(GyroX);
    Serial.print(((const __FlashStringHelper *)(" GyroY: ")));
    Serial.print(GyroY);
    Serial.print(((const __FlashStringHelper *)(" GyroZ: ")));
    Serial.println(GyroZ);
  }
}

void printAccelData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(((const __FlashStringHelper *)("AccX: ")));
    Serial.print(AccX);
    Serial.print(((const __FlashStringHelper *)(" AccY: ")));
    Serial.print(AccY);
    Serial.print(((const __FlashStringHelper *)(" AccZ: ")));
    Serial.println(AccZ);
  }
}

void pjintMagData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(((const __FlashStringHelper *)("MagX: ")));
    Serial.print(MagX);
    Serial.print(((const __FlashStringHelper *)(" MagY: ")));
    Serial.print(MagY);
    Serial.print(((const __FlashStringHelper *)(" MagZ: ")));
    Serial.println(MagZ);
  }
}

void printRollPitchYaw() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(((const __FlashStringHelper *)("roll: ")));
    Serial.print(roll_IMU);
    Serial.print(((const __FlashStringHelper *)(" pitch: ")));
    Serial.print(pitch_IMU);
    Serial.print(((const __FlashStringHelper *)(" yaw: ")));
    Serial.println(yaw_IMU);
  }
}

void printRollPitchYawAndDesired() {
 // Will print in this order:
 // 	(roll) (pitch) (yaw) (thro_des) (roll_des) (pitch_des) (yaw_des)
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(roll_IMU);
  Serial.print("\t");
    Serial.print(pitch_IMU);
  Serial.print("\t");
    Serial.print(yaw_IMU);
  Serial.print("\t");
    Serial.print(thro_des);
  Serial.print("\t");
    Serial.print(roll_des);
  Serial.print("\t");
    Serial.print(pitch_des);
  Serial.print("\t");
    Serial.println(yaw_des);
  }
}

void printPIDoutput() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(((const __FlashStringHelper *)("roll_PID: ")));
    Serial.print(roll_PID);
    Serial.print(((const __FlashStringHelper *)(" pitch_PID: ")));
    Serial.print(pitch_PID);
    Serial.print(((const __FlashStringHelper *)(" yaw_PID: ")));
    Serial.print(yaw_PID);
    Serial.print(((const __FlashStringHelper *)(" roll_PID_new: ")));
    Serial.print(pidOutputVals[2]);
    Serial.print(((const __FlashStringHelper *)(" pitch_PID_new: ")));
    Serial.print(pidOutputVals[0]);
    Serial.print(((const __FlashStringHelper *)(" yaw_PID_new: ")));
    Serial.println(pidOutputVals[1]);
  }
}

void printMotorCommands() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(((const __FlashStringHelper *)("m1_command: ")));
    Serial.print(m1_command_PWM);
    Serial.print(((const __FlashStringHelper *)(" m2_command: ")));
    Serial.print(m2_command_PWM);
    Serial.print(((const __FlashStringHelper *)(" m3_command: ")));
    Serial.print(m3_command_PWM);
    Serial.print(((const __FlashStringHelper *)(" m4_command: ")));
    Serial.print(m4_command_PWM);
    Serial.print(((const __FlashStringHelper *)(" m5_command: ")));
    Serial.print(m5_command_PWM);
    Serial.print(((const __FlashStringHelper *)(" m6_command: ")));
    Serial.println(m6_command_PWM);
  }
}

void printServoCommands() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(((const __FlashStringHelper *)("s1_command: ")));
    Serial.print(s1_command_PWM);
    Serial.print(((const __FlashStringHelper *)(" s2_command: ")));
    Serial.print(s2_command_PWM);
    Serial.print(((const __FlashStringHelper *)(" s3_command: ")));
    Serial.print(s3_command_PWM);
    Serial.print(((const __FlashStringHelper *)(" s4_command: ")));
    Serial.print(s4_command_PWM);
    Serial.print(((const __FlashStringHelper *)(" s5_command: ")));
    Serial.print(s5_command_PWM);
    Serial.print(((const __FlashStringHelper *)(" s6_command: ")));
    Serial.print(s6_command_PWM);
    Serial.print(((const __FlashStringHelper *)(" s7_command: ")));
    Serial.println(s7_command_PWM);
  }
}

void printLoopRate() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(((const __FlashStringHelper *)("dt = ")));
    Serial.println(dt*1000000.0);
  }
}

void getJoyAngle() {
 alphaCounts = analogRead(joyAlphaPin);
 betaCounts = analogRead(joyBetaPin);
 alpha = alphaCounts*0.06577f - 40.0f;
 beta = betaCounts*(-0.05971f) + 36.0f;
}

void openIris() {
 iris.write(60);
 servoLoopCounter = 0;
}

void closeIris() {
 if (servoLoopCounter < 500) {
  iris.write(138);
  servoLoopCounter++;
 } else {
  iris.write(135);
 }
}

void calibrateJoystick() {
 alphaCounts_max = 0;
 alphaCounts_min = 1000;
 betaCounts_max = 0;
 betaCounts_min = 1000;

 while (1) {
  alphaCounts = analogRead(joyAlphaPin);
  betaCounts = analogRead(joyBetaPin);

  if (alphaCounts < alphaCounts_min) {
   alphaCounts_min = alphaCounts;
  }
  if (alphaCounts > alphaCounts_max) {
   alphaCounts_max = alphaCounts;
  }
  if (betaCounts < betaCounts_min) {
   betaCounts_min = betaCounts;
  }
  if (betaCounts > betaCounts_max) {
   betaCounts_max = betaCounts;
  }
  Serial.print("alphaCounts_max = ");
  Serial.print(alphaCounts_max);
  Serial.print("\t");
  Serial.print("alphaCounts_min = ");
  Serial.print(alphaCounts_min);
  Serial.print("\t");
  Serial.print("betaCounts_max = ");
  Serial.print(betaCounts_max);
  Serial.print("\t");
  Serial.print("betaCounts_min = ");
  Serial.println(betaCounts_min);
 }
}

void printRIPAngles() {
 if (current_time - print_counter > 10000) {
  print_counter = micros();
  //Serial.print("Alpha: ");
  Serial.print(alpha);
  Serial.print(" ");
  //Serial.print(" Roll: ");
  Serial.print(roll_IMU);
  Serial.print(" ");
  //Serial.print(" Alpha + Roll: ");
  Serial.print(alpha + roll_IMU);
  Serial.print(" ");
  //Serial.print(" Beta: ");
  Serial.print(beta);
  Serial.print(" ");
  //Serial.print(" Pitch: ");
  Serial.print(pitch_IMU);
  Serial.print(" ");
  //Serial.print(" Beta + Pitch: ");
  Serial.println(beta + pitch_IMU);
  //Serial.print("AlphaCounts: ");
  //Serial.print(alphaCounts);
  //Serial.print(" ");
  //Serial.print("BetaCounts: ");
  //Serial.println(betaCounts);
 }
}

String getDataString() {
 String csvDataString;
 csvDataString = String(roll_IMU)
         + ","
         + String(pitch_IMU)
         + ","
         + String(yaw_IMU)
         + ","
         + String(alpha)
         + ","
         + String(beta)
         + ","
         + String(roll_des)
         + ","
         + String(pitch_des)
         + ","
         + String(yaw_des)
         + ","
         + String(thro_des)
         + ","
         + String(roll_PID)
         + ","
         + String(pitch_PID)
         + ","
         + String(yaw_PID)
         + ","
         + String(channel_1_pwm)
         + ","
         + String(channel_2_pwm)
         + ","
         + String(channel_3_pwm)
         + ","
         + String(channel_4_pwm)
         + ","
         + String(channel_5_pwm)
         + ","
         + String(channel_6_pwm)
         + ","
         + String(channel_7_pwm)
         + ","
         + String(channel_8_pwm)
         + ","
         + String(channel_9_pwm)
         + ","
         + String(channel_10_pwm)
         + ","
         + String(channel_11_pwm)
         + ","
         + String(channel_12_pwm)
         + ","
         + String(channel_13_pwm)
         + ","
         + String(GyroX)
         + ","
         + String(GyroY)
         + ","
         + String(GyroZ)
         + ","
         + String(AccX)
         + ","
         + String(AccY)
         + ","
         + String(AccZ)
         + ","
         + String(s1_command_scaled)
         + ","
         + String(s2_command_scaled)
         + ","
         + String(s3_command_scaled)
         + ","
         + String(s4_command_scaled)
         + ","
         + String(P_gains(2,2)*P_gainScale(2,2))
         + ","
         + String(I_gains(2,2)*I_gainScale(2,2))
         + ","
         + String(D_gains(2,2)*D_gainScale(2,2))
         + ","
         + String(P_gains(0,0)*P_gainScale(0,0))
         + ","
         + String(I_gains(0,0)*I_gainScale(0,0))
         + ","
         + String(D_gains(0,0)*D_gainScale(0,0))
         + ","
         + String(P_gains(1,1)*P_gainScale(1,1))
         + ","
         + String(I_gains(1,1)*I_gainScale(1,1))
         + ","
         + String(D_gains(1,1)*D_gainScale(1,1))
         + ","
         + String(failureFlag);
 return csvDataString;
}

void displayRoll() {
 if (current_time - print_counter > 10000) {
  print_counter = micros();
  Serial.print(roll_des);
  Serial.print(" ");
  Serial.println(roll_IMU);
 }
}

void displayPitch() {
 if (current_time - print_counter > 10000) {
  print_counter = micros();
  Serial.print(pitch_des);
  Serial.print(" ");
  Serial.println(pitch_IMU);
 }
}

void getPScale() {
 float scaleVal;
 scaleVal = 1.0f + (channel_10_pwm - 1500.0f)/500.0f * 0.8f;
 P_gainScale(0,0) = scaleVal;
 P_gainScale(2,2) = scaleVal;
}

void getDScale() {
 float scaleVal;
 scaleVal = 1.0f + (channel_12_pwm - 1500.0f)/500.0f * 0.8f;
 D_gainScale(0,0) = scaleVal;
 D_gainScale(2,2) = scaleVal;

}

void getIScale() {
 float scaleVal;
 scaleVal = 1.0f + (channel_11_pwm - 1500.0f)/500.0f * 0.8f;
 I_gainScale(0,0) = scaleVal;
 I_gainScale(2,2) = scaleVal;
}

void rollGainOffset() {
 float offsetVal;
 offsetVal = 1.0f + (channel_13_pwm - 1500.0f)/500.0f * 0.05f;
 P_gainScale(0,0) *= offsetVal;
 P_gainScale(2,2) *= offsetVal;
 I_gainScale(0,0) *= offsetVal;
 I_gainScale(2,2) *= offsetVal;
 D_gainScale(0,0) *= offsetVal;
 D_gainScale(2,2) *= offsetVal;

}

void anglePID() {

  // --- Roll --- //
  float error_roll = roll_des - roll_IMU;
  float integral_roll = integralOld_roll + error_roll*dt;
  if (channel_1_pwm < 1060) { //Don't let integrator build if throttle is too low
    integral_roll = 0;
  }
  //Saturate integrator to prevent unsafe buildup
  integral_roll = ({ typeof(integral_roll) _amt = (integral_roll); typeof(-i_limit) _low = (-i_limit); typeof(i_limit) _high = (i_limit); (_amt < _low) ? _low : ((_amt > _high) ? _high : _amt); });
  float derivative_roll = GyroX;



 //Scaled by .01 to bring within -1 to 1 range
  roll_PID = 0.01*(Kp_roll_angle*error_roll
            + Ki_roll_angle*integral_roll
          - Kd_roll_angle*derivative_roll);

  // --- Pitch --- //
  float error_pitch = pitch_des - pitch_IMU;
  float integral_pitch = integralOld_pitch + error_pitch*dt;
  if (channel_1_pwm < 1060) {
    integral_pitch = 0;
  }
  integral_pitch = ({ typeof(integral_pitch) _amt = (integral_pitch); typeof(-i_limit) _low = (-i_limit); typeof(i_limit) _high = (i_limit); (_amt < _low) ? _low : ((_amt > _high) ? _high : _amt); });
  float derivative_pitch = GyroY;



 pitch_PID = 0.01*(Kp_pitch_angle*error_pitch
          + Ki_pitch_angle*integral_pitch
          - Kd_pitch_angle*derivative_pitch);

  // --- Yaw --- // stablize on rate from GyroZ
  float error_yaw = yaw_des - GyroZ;
  float integral_yaw = integralOld_yaw + error_yaw*dt;
  if (channel_1_pwm < 1060) {
    integral_yaw = 0;
  }
  integral_yaw = ({ typeof(integral_yaw) _amt = (integral_yaw); typeof(-i_limit) _low = (-i_limit); typeof(i_limit) _high = (i_limit); (_amt < _low) ? _low : ((_amt > _high) ? _high : _amt); });
  float derivative_yaw = (error_yaw - errorOld_yaw)/dt;



  yaw_PID = 0.01*(Kp_yaw*error_yaw
         + Ki_yaw*integral_yaw
         + Kd_yaw*derivative_yaw);

  //Update roll variables
  integralOld_roll = integral_roll;
  //Update pitch variables
  integralOld_pitch = integral_pitch;
  //Update yaw variables
  errorOld_yaw = error_yaw;
  integralOld_yaw = integral_yaw;
}


//=========================================================================================//

//HELPER FUNCTIONS

float invSqrt(float x) {
  //Fast inverse sqrt for madgwick filter
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
# 2513 "/home/james/Documents/dRehmFlight/Versions/dRehmFlight_quad/dRehmFlight_quad.ino"
  /*

  //alternate form:

  unsigned int i = 0x5F1F1412 - (*(unsigned int*)&x >> 1);

  float tmp = *(float*)&i;

  float y = tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);

  return y;

  */
# 2520 "/home/james/Documents/dRehmFlight/Versions/dRehmFlight_quad/dRehmFlight_quad.ino"
  return 1.0/sqrtf(x); //Teensy is fast enough to just take the compute penalty lol suck it arduino nano
}
# 1 "/home/james/Documents/dRehmFlight/Versions/dRehmFlight_quad/radioComm.ino"
//Arduino/Teensy Flight Controller - dRehmFlight
//Author: Nicholas Rehm
//Project Start: 1/6/2020
//Last Updated: 7/29/2022
//Version: Beta 1.3

//========================================================================================================================//

//This file contains all necessary functions and code used for radio communication to avoid cluttering the main code

unsigned long rising_edge_start_1, rising_edge_start_2, rising_edge_start_3, rising_edge_start_4,
        rising_edge_start_5, rising_edge_start_6, rising_edge_start_7;
unsigned long channel_1_raw, channel_2_raw, channel_3_raw, channel_4_raw, channel_5_raw,
       channel_6_raw, channel_7_raw, channel_8_raw, channel_9_raw;
int ppm_counter = 0;
unsigned long time_ms = 0;

void radioSetup() {
  //PPM Receiver 
# 48 "/home/james/Documents/dRehmFlight/Versions/dRehmFlight_quad/radioComm.ino"
    sbus.begin();

  //DSM receiver





}

unsigned long getRadioPWM(int ch_num) {
  //DESCRIPTION: Get current radio commands from interrupt routines 
  unsigned long returnPWM = 0;

  if (ch_num == 1) {
    returnPWM = channel_1_raw;
  }
  else if (ch_num == 2) {
    returnPWM = channel_2_raw;
  }
  else if (ch_num == 3) {
    returnPWM = channel_3_raw;
  }
  else if (ch_num == 4) {
    returnPWM = channel_4_raw;
  }
  else if (ch_num == 5) {
    returnPWM = channel_5_raw;
  }
  else if (ch_num == 6) {
    returnPWM = channel_6_raw;
  }

  return returnPWM;
}

//For DSM type receivers
void serialEvent3(void)
{





}



//========================================================================================================================//



//INTERRUPT SERVICE ROUTINES (for reading PWM and PPM)

void getPPM() {
  unsigned long dt_ppm;
  int trig = digitalRead(PPM_Pin);
  if (trig==1) { //Only care about rising edge
    dt_ppm = micros() - time_ms;
    time_ms = micros();


    if (dt_ppm > 5000) { //Waiting for long pulse to indicate a new pulse train has arrived
      ppm_counter = 0;
    }

    if (ppm_counter == 1) { //First pulse
      channel_1_raw = dt_ppm;
    }

    if (ppm_counter == 2) { //Second pulse
      channel_2_raw = dt_ppm;
    }

    if (ppm_counter == 3) { //Third pulse
      channel_3_raw = dt_ppm;
    }

    if (ppm_counter == 4) { //Fourth pulse
      channel_4_raw = dt_ppm;
    }

    if (ppm_counter == 5) { //Fifth pulse
      channel_5_raw = dt_ppm;
    }

    if (ppm_counter == 6) { //Sixth pulse
      channel_6_raw = dt_ppm;
    }

    ppm_counter = ppm_counter + 1;
  }
}

void getCh1() {
  int trigger = digitalRead(ch1Pin);
  if(trigger == 1) {
    rising_edge_start_1 = micros();
  }
  else if(trigger == 0) {
    channel_1_raw = micros() - rising_edge_start_1;
  }
}

void getCh2() {
  int trigger = digitalRead(ch2Pin);
  if(trigger == 1) {
    rising_edge_start_2 = micros();
  }
  else if(trigger == 0) {
    channel_2_raw = micros() - rising_edge_start_2;
  }
}

void getCh3() {
  int trigger = digitalRead(ch3Pin);
  if(trigger == 1) {
    rising_edge_start_3 = micros();
  }
  else if(trigger == 0) {
    channel_3_raw = micros() - rising_edge_start_3;
  }
}

void getCh4() {
  int trigger = digitalRead(ch4Pin);
  if(trigger == 1) {
    rising_edge_start_4 = micros();
  }
  else if(trigger == 0) {
    channel_4_raw = micros() - rising_edge_start_4;
  }
}

void getCh5() {
  int trigger = digitalRead(ch5Pin);
  if(trigger == 1) {
    rising_edge_start_5 = micros();
  }
  else if(trigger == 0) {
    channel_5_raw = micros() - rising_edge_start_5;
  }
}

void getCh6() {
  int trigger = digitalRead(ch6Pin);
  if(trigger == 1) {
    rising_edge_start_6 = micros();
  }
  else if(trigger == 0) {
    channel_6_raw = micros() - rising_edge_start_6;
  }
}