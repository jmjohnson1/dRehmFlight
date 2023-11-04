#include <RingBuf.h>
#include <SdFat.h>
#include "commonDefinitions.h"


#define USE_MPU6050_I2C

#if defined USE_MPU6050_I2C
#include "src/MPU6050/MPU6050.h"
MPU6050 imu;
#elif defined USE_MPU9250_SPI
#include "src/MPU9250/MPU9250.h"
MPU9250 mpu9250(SPI2, 36);
#else
#error No MPU defined...
#endif

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

attInfo imu_info;

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
#define LOG_FILE_SIZE 256 * 100 * 600 * 10 // ~1,500,000,000 bytes.

// Space to hold more than 1 second of 256-byte lines at 100 Hz in the buffer
#define RING_BUF_CAPACITY 50 * 512
#define LOG_FILENAME "SdioLogger.csv"

SdFs sd;
FsFile file;

bool sd_is_present = false;

unsigned long current_time, prev_time, print_counterSD;

// Ring buffer for filetype FsFile (The filemanager that will handle the data stream)
RingBuf<FsFile, RING_BUF_CAPACITY> buffer;

int logData_setup() {
	// Initialize the SD
	if (!sd.begin(SD_CONFIG)) {
		sd.initErrorPrint(&Serial); // Prints message to serial if SD can't init
		return 1;
	}
	// Determine logfile name
	int fileIncrement = 0;
	fileName = filePrefix + String(fileIncrement) + fileExtension;
	while(sd.exists(fileName)) {
		// Increment file name if it exists and try again
		fileIncrement++;
		fileName = filePrefix + String(fileIncrement) + fileExtension;
	}
	// Open or create file - truncate existing
	if (!file.open(fileName.c_str(), O_RDWR | O_CREAT | O_TRUNC)) {
		Serial.println("open failed\n");
		return 1;
	}
	// Initialize ring buffer
	buffer.begin(&file);
	Serial.println("Buffer initialized");
	logData_printCSVHeader();
	return 0;
}

void logData_printCSVHeader() {
    buffer.print("AccX");
    buffer.print(",");
    buffer.print("AccY");
    buffer.print(",");
    buffer.print("AccZ");
    buffer.print(",");
    buffer.print("GyroX");
    buffer.print(",");
    buffer.print("GyroY");
    buffer.print(",");
    buffer.print("GyroZ");
	buffer.println();
}

int logData_writeBuffer() {
	size_t amtDataInBuf = buffer.bytesUsed();
	if ((amtDataInBuf + file.curPosition()) > (LOG_FILE_SIZE - 20)) {
		Serial.println("Log file full -- No longer writing");
		return 1;
	}
	if (amtDataInBuf >= 512 && !file.isBusy()) {
		// One sector (512 bytes) can be printed before busy wait
		// Write from buffer to file
		if (512 != buffer.writeOut(512)) {
			Serial.println("Write to file from buffer failed -- breaking");
			return 1;
		}
	}

    buffer.print()
	buffer.println();

	if (buffer.getWriteError()) {
		Serial.println("WriteError");
		return 1;
	}
	return 0;
}

void logData_endProcess() {
	// Write any remaining buffer data to file
	buffer.sync();
	file.truncate();
	file.rewind();
	file.close();
	
	// DEBUG
	Serial.println("logging ended peacefully");
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
  for (;;);
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

void setup() {
    sd_is_present = !logData_setup();
    IMUinit(&imu);
    imu_info.AccErrorX = 0.0;
    imu_info.AccErrorY = 0.0;
    imu_info.AccErrorZ = 0.0;
    imu_info.GyroErrorX = 0.0;
    imu_info.GyroErrorY = 0.0;
    imu_info.GyroErrorZ = 0.0;
}

void loop() {
    prev_time = current_time;
    current_time = micros();
    float dt = (current_time - prev_time)/1000000.0f;

    getIMUData(&imu_info, &imu); // Pulls raw gyro, accelerometer, and magnetometer data from IMU and LP filters to remove noise
    Madgwick6DOF(&imu_info, dt); // Updates roll_IMU, pitch_IMU, and yaw_IMU angle estimates (degrees)
    
    if (sd_is_present && (current_time - print_counterSD) > LOG_INTERVAL_USEC) {
        print_counterSD = micros();
        logData_writeBuffer();
        Serial.println("logged");
    }

    delay(500);
}
