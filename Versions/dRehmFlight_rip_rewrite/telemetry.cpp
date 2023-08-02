#include "telemetry.h"
#include "src/mavlink/common/mavlink_msg_attitude.h"

// System information
uint8_t sysid = 1;
uint8_t compid_core = 158;
uint8_t compid_rip = 159;
uint8_t systype = MAV_TYPE_QUADROTOR;
uint8_t autopilotType = MAV_AUTOPILOT_INVALID;
uint8_t systemMode = MAV_MODE_PREFLIGHT;
uint8_t customMode = 0;
uint8_t systemState = MAV_STATE_STANDBY;

void InitTelemetry() {
  // Mavlink serial ports. See Teensy 4.1 pinouts for a list of available
  // hardware serial ports. Defined in the header file.
  HWSERIAL.begin(57600);
}

void SendMessage(uint8_t *msg_buf, mavlink_message_t *msg) {
	// Copy message to send buffer
	uint16_t len = mavlink_msg_to_send_buffer(msg_buf, msg);
	// Send message
	HWSERIAL.write(msg_buf, len);
}

void SendHeartbeat() {
    // Buffer
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // Pack message
    mavlink_msg_heartbeat_pack(sysid, compid_core, &msg, systype, autopilotType,
                               systemMode, customMode, systemState);

    // Copy message to send buffer
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    // Send the message with UART
    HWSERIAL.write(buf, len);
}

void SendPIDGains_core(float P, float I, float D) {
	// Buffer
  mavlink_message_t pid_msg;
	uint8_t pid_buf[MAVLINK_MAX_PACKET_LEN];

	// Pack message
  mavlink_msg_pid_tuning_pack(sysid, compid_core, &pid_msg, 0.0f, 0.0f, 0.0f, 0.0f,
                              P, I, D, 0.0f, 0.0f);
	// Copy message to send buffer
	uint16_t len = mavlink_msg_to_send_buffer(pid_buf, &pid_msg);

	// Send message
	HWSERIAL.write(pid_buf, len);
}

void SendPIDGains_rip(float P, float I, float D) {
	// Buffer
  mavlink_message_t pid_msg;
	uint8_t pid_buf[MAVLINK_MAX_PACKET_LEN];

	// Pack message
  mavlink_msg_pid_tuning_pack(sysid, compid_rip, &pid_msg, 0.0f, 0.0f, 0.0f, 0.0f,
                              P, I, D, 0.0f, 0.0f);
	// Copy message to send buffer
	uint16_t len = mavlink_msg_to_send_buffer(pid_buf, &pid_msg);

	// Send message
	HWSERIAL.write(pid_buf, len);
}

void SendAttitude(float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed) {
	// Buffer
	mavlink_message_t att_msg;
	uint8_t msg_buf[MAVLINK_MAX_PACKET_LEN];

	// Pack Message
	mavlink_msg_attitude_pack(sysid, compid_core, &att_msg, 0, roll, pitch, yaw, rollspeed, pitchspeed, yawspeed);

	// Copy message to send buffer
	uint16_t len = mavlink_msg_to_send_buffer(msg_buf, &att_msg);

	// Send message
	HWSERIAL.write(msg_buf, len);
}


