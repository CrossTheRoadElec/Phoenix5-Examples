#include <Tasks/TaskHSV.h>
#include "ctre/phoenix/HsvToRgb.h"
#include "Platform/Platform.h"

TaskHSV::~TaskHSV() {}
/* ILoopable */
void TaskHSV::OnStart() {
}
void TaskHSV::OnStop() {
}
bool TaskHSV::IsDone() {
	return false;
}
void TaskHSV::OnLoop() {
	if (Saturation > 1) {
		Saturation = 1;
	}
	if (Saturation < 0)
		Saturation = 0;

	if (Value > 1)
		Value = 1;
	if (Value < 0)
		Value = 0;

	/* convert to rgb */
	CTRE::HsvToRgb::Convert(Hue, Saturation, Value, &_r, &_g, &_b);

	_r = _averageR->Process(_r);
	_g = _averageG->Process(_g);
	_b = _averageB->Process(_b);

	/* update CANifier's LED strip */
	Hardware::canifier->SetLEDOutput(_r,
			CTRE::CANifier::LEDChannel::LEDChannelA);
	Hardware::canifier->SetLEDOutput(_g,
			CTRE::CANifier::LEDChannel::LEDChannelB);
	Hardware::canifier->SetLEDOutput(_b,
			CTRE::CANifier::LEDChannel::LEDChannelC);
}
