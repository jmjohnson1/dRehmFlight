void printRadioData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F(" CH1: "));
    Serial.print(channel_1_pwm);
    Serial.print(F(" CH2: "));
    Serial.print(channel_2_pwm);
    Serial.print(F(" CH3: "));
    Serial.print(channel_3_pwm);
    Serial.print(F(" CH4: "));
    Serial.print(channel_4_pwm);
    Serial.print(F(" CH5: "));
    Serial.print(channel_5_pwm);
    Serial.print(F(" CH6: "));
    Serial.print(channel_6_pwm);
    Serial.print(F(" CH7: "));
    Serial.print(channel_7_pwm);
    Serial.print(F(" CH8: "));
    Serial.print(channel_8_pwm);
    Serial.print(F(" CH9: "));
    Serial.print(channel_9_pwm);
    Serial.print(F(" CH10: "));
    Serial.print(channel_10_pwm);
    Serial.print(F(" CH11: "));
    Serial.print(channel_11_pwm);
    Serial.print(F(" CH12: "));
    Serial.print(channel_12_pwm);
    Serial.print(F(" CH13: "));
    Serial.println(channel_13_pwm);
  }
}

void printDesiredState() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("thro_des: "));
    Serial.print(thro_des);
    Serial.print(F(" roll_des: "));
    Serial.print(roll_des);
    Serial.print(F(" pitch_des: "));
    Serial.print(pitch_des);
    Serial.print(F(" yaw_des: "));
    Serial.println(yaw_des);
  }
}

void printGyroData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("GyroX: "));
    Serial.print(quadIMU_info.GyroX);
    Serial.print(F(" GyroY: "));
    Serial.print(quadIMU_info.GyroY);
    Serial.print(F(" GyroZ: "));
    Serial.println(quadIMU_info.GyroZ);
  }
}

void printAccelData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("AccX: "));
    Serial.print(quadIMU_info.AccX);
    Serial.print(F(" AccY: "));
    Serial.print(quadIMU_info.AccY);
    Serial.print(F(" AccZ: "));
    Serial.println(quadIMU_info.AccZ);
  }
}

void printMagData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("MagX: "));
    Serial.print(quadIMU_info.MagX);
    Serial.print(F(" MagY: "));
    Serial.print(quadIMU_info.MagY);
    Serial.print(F(" MagZ: "));
    Serial.println(quadIMU_info.MagZ);
  }
}

void printRollPitchYaw() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("roll: "));
    Serial.print(quadIMU_info.roll_IMU);
    Serial.print(F(" pitch: "));
    Serial.print(quadIMU_info.pitch_IMU);
    Serial.print(F(" yaw: "));
    Serial.println(quadIMU_info.yaw_IMU);
  }
}

void printRollPitchYawAndDesired() {
  // Will print in this order:
  // 	(roll) (pitch) (yaw) (thro_des) (roll_des) (pitch_des) (yaw_des)
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(quadIMU_info.roll_IMU);
    Serial.print("\t");
    Serial.print(quadIMU_info.pitch_IMU);
    Serial.print("\t");
    Serial.print(quadIMU_info.yaw_IMU);
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
    Serial.print(F("roll_PID: "));
    Serial.print(roll_PID);
    Serial.print(F(" pitch_PID: "));
    Serial.print(pitch_PID);
    Serial.print(F(" yaw_PID: "));
    Serial.println(yaw_PID);
  }
}

void printMotorCommands() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("m1_command: "));
    Serial.print(m1_command_PWM);
    Serial.print(F(" m2_command: "));
    Serial.print(m2_command_PWM);
    Serial.print(F(" m3_command: "));
    Serial.print(m3_command_PWM);
    Serial.print(F(" m4_command: "));
    Serial.print(m4_command_PWM);
    Serial.print(F(" m5_command: "));
    Serial.print(m5_command_PWM);
    Serial.print(F(" m6_command: "));
    Serial.println(m6_command_PWM);
  }
}

void printServoCommands() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("s1_command: "));
    Serial.print(s1_command_PWM);
    Serial.print(F(" s2_command: "));
    Serial.print(s2_command_PWM);
    Serial.print(F(" s3_command: "));
    Serial.print(s3_command_PWM);
    Serial.print(F(" s4_command: "));
    Serial.print(s4_command_PWM);
    Serial.print(F(" s5_command: "));
    Serial.print(s5_command_PWM);
    Serial.print(F(" s6_command: "));
    Serial.print(s6_command_PWM);
    Serial.print(F(" s7_command: "));
    Serial.println(s7_command_PWM);
  }
}

void printLoopRate() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("dt = "));
    Serial.println(dt * 1000000.0);
  }
}

void printRIPAngles() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    // Serial.print("Alpha: ");
    Serial.print(joyRoll);
    Serial.print(" ");
    // Serial.print(" Roll: ");
    Serial.print(quadIMU_info.roll_IMU);
    Serial.print(" ");
    // Serial.print(" Alpha + Roll: ");
    Serial.print(joyRoll + quadIMU_info.roll_IMU);
    Serial.print(" ");
    // Serial.print(" Beta: ");
    Serial.print(joyPitch);
    Serial.print(" ");
    // Serial.print(" Pitch: ");
    Serial.print(quadIMU_info.pitch_IMU);
    Serial.print(" ");
    // Serial.print(" Beta + Pitch: ");
    Serial.println(joyPitch + quadIMU_info.pitch_IMU);
    // Serial.print("AlphaCounts: ");
    // Serial.print(alphaCounts);
    // Serial.print(" ");
    // Serial.print("BetaCounts: ");
    // Serial.println(betaCounts);
  }
}

void displayRoll() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(roll_des);
    Serial.print(" ");
    Serial.println(quadIMU_info.roll_IMU);
  }
}

void displayPitch() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(pitch_des);
    Serial.print(" ");
    Serial.print(quadIMU_info.pitch_IMU);
    Serial.println();
  }
}

void printPIDGains() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print("Kp_pitch: ");
    Serial.print(Kp_roll_angle);
    Serial.print(" Ki_pitch: ");
    Serial.print(Ki_pitch_angle);
    Serial.print(" Kd_pitch: ");
    Serial.print(Kd_pitch_angle);
    Serial.println();
  }
}
void printRIPIMUData() {
	if (current_time - print_counter > 10000) {
		print_counter = micros();

		Serial.print("Roll: ");
		Serial.print(ripIMU_info.roll_IMU);
		
		Serial.print(" Pitch: ");
		Serial.print(ripIMU_info.pitch_IMU);

		Serial.print(" Yaw: ");
		Serial.print(ripIMU_info.yaw_IMU);

		Serial.println();
	}
}

void displayRIPCombo() {
	if (current_time - print_counter > 10000) {
		print_counter = micros();

		Serial.print(joyRoll);
		Serial.print(" ");
		Serial.print(ripRoll);
		Serial.print(" ");
		Serial.print(joyPitch);
		Serial.print(" ");
		Serial.print(ripPitch);
		Serial.print(" ");
		Serial.print(ripIMU_info.roll_IMU);
		Serial.print(" ");
		Serial.print(ripIMU_info.pitch_IMU);
		
		Serial.println();
	}
}

void displayResponse() {
	if (current_time - print_counter > 10000) {
		Serial.print(alphaRoll_des);
		Serial.print(" ");
		Serial.print(quadIMU_info.roll_IMU);
		Serial.println();
	}
}

