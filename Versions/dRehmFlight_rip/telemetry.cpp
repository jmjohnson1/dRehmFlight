#include "telemetry.h"

// Global variables
unsigned long previousMillis_heartbeat = 0;
unsigned long previousMillis_corePID = 0;
unsigned long previousMillis_ripPID = 0;
unsigned long nextIntervalMavlink = 1000;

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

void SendHeartbeat() {
  unsigned long currentMillisMavlink = millis();
  if (currentMillisMavlink - previousMillis_heartbeat >= nextIntervalMavlink) {
    previousMillis_heartbeat = currentMillisMavlink;

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
}

void SendPIDGains_core(float P, float I, float D) {
  unsigned long currentMillisMavlink = millis();
  if (currentMillisMavlink - previousMillis_corePID >= nextIntervalMavlink) {
    previousMillis_corePID = currentMillisMavlink;
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
}

void SendPIDGains_rip(float P, float I, float D) {
  unsigned long currentMillisMavlink = millis();
  if (currentMillisMavlink - previousMillis_ripPID >= nextIntervalMavlink) {
    previousMillis_ripPID = currentMillisMavlink;
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
}

