#include "src/MPU6050/MPU6050.h"

#define ACCEL_SCALE_FACTOR 16384.0
#define GYRO_SCALE_FACTOR 131.0

typedef struct imuInfo {
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
  float B_gyro = 0.1;
  float B_accel = 0.14;
  float B_madgwick = 0.04f;
} imuInfo;

typedef struct attitude {
  float roll_IMU;
  float pitch_IMU;
  float yaw_IMU;
} attitude;

imuInfo ripIMU_info;
attitude ripAtt;
MPU6050 ripIMU;
unsigned long current_time, prev_time;
float dt;

void setup() {
  Serial.begin(500000);
  IMUinit();
  // calculate_IMU_error(&ripIMU_info, &ripIMU);
  ripIMU_info.AccErrorX = 1.03;
  ripIMU_info.AccErrorY = 0.02;
  ripIMU_info.AccErrorZ = -1.05;
  ripIMU_info.GyroErrorX = -3.45;
  ripIMU_info.GyroErrorY = -0.22;
  ripIMU_info.GyroErrorZ = 0.90;
}

void loop() {
  prev_time = current_time;
  current_time = micros();
  dt = (current_time - prev_time) / 1000000.0;

  Serial.print("Roll = ");
  Serial.print(ripAtt.roll_IMU);
  Serial.print(" Pitch = ");
  Serial.print(ripAtt.pitch_IMU);
  Serial.print(" Yaw = ");
  Serial.print(ripAtt.yaw_IMU);
  Serial.print(" dt = ");
  Serial.print(dt, 10);
  Serial.println();

  getIMUData(&ripIMU_info);
  Madgwick6DOF(&ripAtt, &ripIMU_info, dt);

  loopRate(2000); // Do not exceed 2000Hz, all filter parameters tuned to 2000Hz
}

void loopRate(int freq) {
  float invFreq = 1.0 / freq * 1000000.0;
  unsigned long checker = micros();

  // Sit in loop until appropriate time has passed
  while (invFreq > (checker - current_time)) {
    checker = micros();
  }
}

void Madgwick6DOF(attitude *att, imuInfo *imu, float invSampleFreq) {
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
  att->yaw_IMU = atan2(imu->q0 * imu->q1 + imu->q2 * imu->q3, 0.5f - imu->q1 * imu->q1 - imu->q2 * imu->q2) *
                 57.29577951;                                                           // degrees
  att->pitch_IMU = asin(-2.0f * (imu->q1 * imu->q3 - imu->q0 * imu->q2)) * 57.29577951; // degrees
  att->roll_IMU = -atan2(imu->q1 * imu->q2 + imu->q0 * imu->q3, 0.5f - imu->q2 * imu->q2 - imu->q3 * imu->q3) *
                  57.29577951; // degrees
}

float invSqrt(float x) { return 1.0f / sqrtf(x); }

void IMUinit() {
  Wire.begin();
  Wire.setClock(400000);

  ripIMU.initialize();

  if (ripIMU.testConnection() == false) {
    Serial.println("MPU6050 init unsuccessful");
    for (;;)
      ;
  }

  ripIMU.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  ripIMU.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
}

void getIMUData(imuInfo *imu) {
  int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
  ripIMU.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);

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

void calculate_IMU_error(imuInfo *imu, MPU6050 *mpu6050) {
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
