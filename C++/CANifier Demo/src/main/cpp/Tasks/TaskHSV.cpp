#include "Tasks/TaskHSV.h"
#include "ctre/Phoenix.h"
#include "Platform/Platform.h"

/* Destructor */
TaskHSV::~TaskHSV() { }

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

	/* Convert to RGB */
	ctre::phoenix::HsvToRgb::Convert(Hue, Saturation, Value, &_r, &_g, &_b);

	_r = _averageR->Process(_r);
	_g = _averageG->Process(_g);
	_b = _averageB->Process(_b);

	/* Update CANifier's LED strip */
	Hardware::canifier->SetLEDOutput(_r, CANifier::LEDChannel::LEDChannelA);
	Hardware::canifier->SetLEDOutput(_g, CANifier::LEDChannel::LEDChannelB);
	Hardware::canifier->SetLEDOutput(_b, CANifier::LEDChannel::LEDChannelC);
}
