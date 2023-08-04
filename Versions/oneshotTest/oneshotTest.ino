#include "TeensyTimerTool.h"

TeensyTimerTool::OneShotTimer m1_timer(TeensyTimerTool::TMR1);
//TeensyTimerTool::OneShotTimer m2_timer(TeensyTimerTool::TMR1);
//TeensyTimerTool::OneShotTimer m3_timer(TeensyTimerTool::TMR1);
//TeensyTimerTool::OneShotTimer m4_timer(TeensyTimerTool::TMR1);

bool m1_inProgress = false;
bool m2_inProgress = false;
bool m3_inProgress = false;
bool m4_inProgress = false;

int m1_value = 0;
int m2_value = 0;
int m3_value = 0;
int m4_value = 0;

const int m1_pin = 2;
const int m2_pin = 3;
const int m3_pin = 4;
const int m4_pin = 5;

void endM1Pulse() {
	digitalWrite(m1_pin, LOW);
	m1_inProgress = 0;
	Serial.println("m1 ended");
}

void setup() {
	Serial.begin(9600);
	Serial.println("Setting up ...");
	m1_timer.begin(endM1Pulse);
	Serial.println("Setup complete!");
}

void loop() {
	commandMotors();
  float x = 1.0f/sqrt(100000.0f);
	while (m1_inProgress) {
	}
  delayMicroseconds(300);
}

void commandMotors() {
	float m1_length = m1_value*125.0f + 125.0f;


	digitalWrite(m1_pin, HIGH);
	m1_inProgress = true;
	Serial.println("m1 started");

	m1_timer.trigger(m1_length);
}

