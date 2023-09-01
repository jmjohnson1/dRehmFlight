#include "telemetry.h"
#include "src/mavlink/common/mavlink_msg_attitude.h"
#include "src/mavlink/common/mavlink_msg_param_request_list.h"
#include "src/mavlink/mavlink_helpers.h"

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

void Telemetry::UpdateReceived() {
	mavlink_message_t msg;
	mavlink_status_t status;
	int chan = 0;

	while (HWSERIAL.available()) {
		uint8_t byte = HWSERIAL.read();

		if (mavlink_parse_char(chan, byte, &msg, &status)) {
			HandleMessage(&msg);
			Serial.println("Message received");
		}
	}
}

void Telemetry::HandleMessage(mavlink_message_t *msg) {
	Serial.print("Message ID: ");
	Serial.println(msg->msgid);
	switch (msg->msgid) {
		case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
			HandleParamRequest(msg);
			break;
		case MAVLINK_MSG_ID_COMMAND_LONG:
			HandleCommandLong(msg);
			break;
		default:
			break;
	}
}

void Telemetry::HandleParamRequest(mavlink_message_t *msg) {
	
}

void Telemetry::HandleCommandLong(mavlink_message_t *msg) {
	mavlink_command_long_t packet;
	mavlink_msg_command_long_decode(msg, &packet);

	// Serial.print("Packet: ");
	// Serial.println(packet.command);
	// Serial.print("Param 1: ");
	// Serial.println(packet.param1);
	// Serial.print("Param 2: ");
	// Serial.println(packet.param2);
	// Serial.print("Param 3: ");
	// Serial.println(packet.param3);
	// Serial.print("Param 4: ");
	// Serial.println(packet.param4);
	// Serial.print("Param 5: ");
	// Serial.println(packet.param5);
	// Serial.print("Param 6: ");
	// Serial.println(packet.param6);
	// Serial.print("Param 7: ");
	// Serial.println(packet.param7);
}
