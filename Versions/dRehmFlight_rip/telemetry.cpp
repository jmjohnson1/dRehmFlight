#include "telemetry.h"
#include "src/mavlink/common/mavlink_msg_attitude.h"

Telemetry::Telemetry() {
}

void Telemetry::InitTelemetry() {
  // Mavlink serial ports. See Teensy 4.1 pinouts for a list of available
  // hardware serial ports. Defined in the header file.
  HWSERIAL.begin(baudRate);
}

void Telemetry::SendMessage(mavlink_message_t *msg) {
	uint8_t msg_buf[MAVLINK_MAX_PACKET_LEN];
	// Copy message to send buffer
	uint16_t len = mavlink_msg_to_send_buffer(msg_buf, msg);
	// Send message
	HWSERIAL.write(msg_buf, len);
}

void Telemetry::SendHeartbeat() {
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(systemID, componentID_core, &msg, systemType, autopilotType,
                               systemMode, customMode, systemState);
		SendMessage(&msg);
}

void Telemetry::SendPIDGains_core(float P, float I, float D) {
  mavlink_message_t msg;
  mavlink_msg_pid_tuning_pack(systemID, componentID_core, &msg, 0.0f, 0.0f, 0.0f, 0.0f,
                              P, I, D, 0.0f, 0.0f);
	SendMessage(&msg);
}

void Telemetry::SendPIDGains_rip(float P, float I, float D) {
  mavlink_message_t msg;
  mavlink_msg_pid_tuning_pack(systemID, componentID_RIP, &msg, 0.0f, 0.0f, 0.0f, 0.0f,
                              P, I, D, 0.0f, 0.0f);
	SendMessage(&msg);
}

void Telemetry::SendAttitude(float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed) {
	mavlink_message_t msg;

	// Convert to radians and change reference frame
	// The flight controller has Z pointing up, MAVLINK assumes Z points down -> negate y and z
	roll       *=  deg2rad;
	pitch      *= -deg2rad;
	yaw        *= -deg2rad;
	rollspeed  *=  deg2rad;
	pitchspeed *= -deg2rad;
	yawspeed   *= -deg2rad;

	mavlink_msg_attitude_pack(systemID, componentID_core, &msg, 0, roll, pitch, yaw, rollspeed, pitchspeed, yawspeed);
	SendMessage(&msg);
}

void Telemetry::SetSystemMode(uint8_t mode) {
	systemMode = mode;
}

void Telemetry::SetSystemState(uint8_t state) {
	systemState = state;
}
