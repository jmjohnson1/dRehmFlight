#ifndef _TELEM_H_
#define _TELEM_H_

#include "arduino.h"

#define HWSERIAL Serial7

#include "src/mavlink/ardupilotmega/mavlink.h"
#include "src/mavlink/ardupilotmega/mavlink_msg_pid_tuning.h"
#include "src/mavlink/mavlink_helpers.h"
#include "src/mavlink/minimal/mavlink_msg_heartbeat.h"
#include <cstdint>

void InitTelemetry();
void SendHeartbeat();
void SendPIDGains_core(float P, float I, float D);
void SendPIDGains_rip(float P, float I, float D);

#endif
