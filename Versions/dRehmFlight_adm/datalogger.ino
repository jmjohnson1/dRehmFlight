
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
	buffer.print("roll_imu");
	buffer.write(",");
	buffer.print("pitch_imu");
	buffer.write(",");
	buffer.print("yaw_imu");
	buffer.write(",");
	buffer.print("roll_des");
	buffer.write(",");
	buffer.print("pitch_des");
	buffer.write(",");
	buffer.print("yaw_des");
	buffer.write(",");
	buffer.print("throttle_des");
	buffer.write(",");
  buffer.print("roll_PID");
  buffer.write(",");
  buffer.print("pitch_PID");
  buffer.write(",");
  buffer.print("yaw_PID");
  buffer.write(",");
	buffer.print("radio_ch1");
	buffer.write(",");
	buffer.print("radio_ch2");
	buffer.write(",");
	buffer.print("radio_ch3");
	buffer.write(",");
	buffer.print("radio_ch4");
	buffer.write(",");
	buffer.print("radio_ch5");
	buffer.write(",");
	buffer.print("radio_ch6");
	buffer.write(",");
	buffer.print("radio_ch7");
	buffer.write(",");
	buffer.print("radio_ch8");
	buffer.write(",");
	buffer.print("radio_ch9");
	buffer.write(",");
	buffer.print("radio_ch10");
	buffer.write(",");
	buffer.print("radio_ch11");
	buffer.write(",");
	buffer.print("radio_ch12");
	buffer.write(",");
	buffer.print("radio_ch13");
	buffer.write(",");
	buffer.print("GyroX");
	buffer.write(",");
	buffer.print("GyroY");
	buffer.write(",");
	buffer.print("GyroZ");
	buffer.write(",");
	buffer.print("AccX");
	buffer.write(",");
	buffer.print("AccY");
	buffer.write(",");
	buffer.print("AccZ");
	buffer.write(",");
	buffer.print("m1_command");
	buffer.write(",");
	buffer.print("m2_command");
	buffer.write(",");
	buffer.print("m3_command");
	buffer.write(",");
	buffer.print("m4_command");
	buffer.write(",");
	buffer.print("kp_roll");
	buffer.write(",");
	buffer.print("ki_roll");
	buffer.write(",");
	buffer.print("kd_roll");
	buffer.write(",");
	buffer.print("kp_pitch");
	buffer.write(",");
	buffer.print("ki_pitch");
	buffer.write(",");
	buffer.print("kd_pitch");
	buffer.write(",");
	buffer.print("kp_yaw");
	buffer.write(",");
	buffer.print("ki_yaw");
	buffer.write(",");
	buffer.print("kd_yaw");
	buffer.write(",");
	buffer.print("failsafeTriggered");
	buffer.println();
}

int logData_writeBuffer() {
	size_t amtDataInBuf = buffer.bytesUsed();
  // DEBUG
	//Serial.print("Data in buffer: ");
	//Serial.println(amtDataInBuf);
	// end DEBUG
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
	buffer.print(quadIMU_info.roll_IMU, 4);
	buffer.write(",");
	buffer.print(quadIMU_info.pitch_IMU, 4);
	buffer.write(",");
	buffer.print(quadIMU_info.yaw_IMU, 4);
	buffer.write(",");
	buffer.print(roll_des, 4);
	buffer.write(",");
	buffer.print(pitch_des, 4);
	buffer.write(",");
	buffer.print(yaw_des, 4);
	buffer.write(",");
	buffer.print(thro_des, 4);
	buffer.write(",");
	buffer.print(roll_PID, 4);
	buffer.write(",");
	buffer.print(pitch_PID, 4);
	buffer.write(",");
	buffer.print(yaw_PID, 4);
	buffer.write(",");
	buffer.print(channel_1_pwm);
	buffer.write(",");
	buffer.print(channel_2_pwm);
	buffer.write(",");
	buffer.print(channel_3_pwm);
	buffer.write(",");
	buffer.print(channel_4_pwm);
	buffer.write(",");
	buffer.print(channel_5_pwm);
	buffer.write(",");
	buffer.print(channel_6_pwm);
	buffer.write(",");
	buffer.print(channel_7_pwm);
	buffer.write(",");
	buffer.print(channel_8_pwm);
	buffer.write(",");
	buffer.print(channel_9_pwm);
	buffer.write(",");
	buffer.print(channel_10_pwm);
	buffer.write(",");
	buffer.print(channel_11_pwm);
	buffer.write(",");
	buffer.print(channel_12_pwm);
	buffer.write(",");
	buffer.print(channel_13_pwm);
	buffer.write(",");
	buffer.print(quadIMU_info.GyroX, 4);
	buffer.write(",");
	buffer.print(quadIMU_info.GyroY, 4);
	buffer.write(",");
	buffer.print(quadIMU_info.GyroZ, 4);
	buffer.write(",");
	buffer.print(quadIMU_info.AccX, 4);
	buffer.write(",");
	buffer.print(quadIMU_info.AccY, 4);
	buffer.write(",");
	buffer.print(quadIMU_info.AccZ, 4);
	buffer.write(",");
	buffer.print(m1_command_scaled, 4);
	buffer.write(",");
	buffer.print(m2_command_scaled, 4);
	buffer.write(",");
	buffer.print(m3_command_scaled, 4);
	buffer.write(",");
	buffer.print(m4_command_scaled, 4);
	buffer.write(",");
	buffer.print(Kp_roll_angle*pScaleRoll, 4);
	buffer.write(",");
	buffer.print(Ki_roll_angle*iScaleRoll, 4);
	buffer.write(",");
	buffer.print(Kd_roll_angle*dScaleRoll, 4);
	buffer.write(",");
	buffer.print(Kp_pitch_angle*pScalePitch, 4);
	buffer.write(",");
	buffer.print(Ki_pitch_angle*iScalePitch, 4);
	buffer.write(",");
	buffer.print(Kd_pitch_angle*dScalePitch, 4);
	buffer.write(",");
	buffer.print(Kp_yaw*pScaleYaw, 4);
	buffer.write(",");
	buffer.print(Ki_yaw*iScaleYaw, 4);
	buffer.write(",");
	buffer.print(Kd_yaw*dScaleYaw, 4);
	buffer.write(",");
	buffer.print(failureFlag);
	buffer.write(",");
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
