#include <Tasks/TaskLEDStrip.h>
#include "Platform/Hardware.h"

TaskLEDStrip::~TaskLEDStrip() {}
/* ILoopable */
void TaskLEDStrip::OnStart() {
}
void TaskLEDStrip::OnStop() {
}
bool TaskLEDStrip::IsDone() {
	return false;
}
void TaskLEDStrip::OnLoop() {
	_saturation = 1;
	_theta += 1;

	if (_theta >= 360) {
		_theta = 0;
	}

	_saturation *= 3.0f;
	if (_saturation > 1) {
		_saturation = 1;
	}

	/* Convert colors to rgb */
	float r, g, b;
	CTRE::HsvToRgb::Convert(_theta, _saturation, _value, &r, &g, &b);	//What is going on here?

	/* Update Canifier's LED strip */
	Hardware::canifier->SetLEDOutput(r,
			CTRE::CANifier::LEDChannel::LEDChannelA);
	Hardware::canifier->SetLEDOutput(g,
			CTRE::CANifier::LEDChannel::LEDChannelB);
	Hardware::canifier->SetLEDOutput(b,
			CTRE::CANifier::LEDChannel::LEDChannelC);
}
