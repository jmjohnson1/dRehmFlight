
int logData_setup() {
	// Initialize the SD
	if (!sd.begin(SD_CONFIG)) {
		sd.initErrorHalt(&Serial); // Prints message to serial if SD can't init
		return 1;
	}
	// Open or create file - truncate existing
	if (!file.open(LOG_FILENAME, O_RDWR | O_CREAT | O_TRUNC)) { // NOTE: Lookup meaning
		Serial.println("open failed\n");
		return 1;
	}
	// Initialize ring buffer
	buffer.begin(&file);
	Serial.println("Buffer initialized");
	return 0;
}

int logData_writeFile() {
	return 0;
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
	buffer.print(roll_IMU, 4);
	buffer.write(",");
	buffer.print(pitch_IMU, 4);
	buffer.write(",");
	buffer.print(yaw_IMU, 4);
	buffer.write(",");
	buffer.print(alpha, 4);
	buffer.write(",");
	buffer.print(beta, 4);
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
	buffer.print(GyroX, 4);
	buffer.write(",");
	buffer.print(GyroY, 4);
	buffer.write(",");
	buffer.print(GyroZ, 4);
	buffer.write(",");
	buffer.print(AccX, 4);
	buffer.write(",");
	buffer.print(AccY, 4);
	buffer.write(",");
	buffer.print(AccZ, 4);
	buffer.write(",");
	buffer.print(s1_command_scaled, 4);
	buffer.write(",");
	buffer.print(s2_command_scaled, 4);
	buffer.write(",");
	buffer.print(s3_command_scaled, 4);
	buffer.write(",");
	buffer.print(s4_command_scaled, 4);
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
	buffer.println(failureFlag);
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
}
