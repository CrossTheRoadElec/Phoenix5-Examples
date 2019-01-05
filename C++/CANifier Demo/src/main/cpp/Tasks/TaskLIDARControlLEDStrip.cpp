#include "Tasks/TaskLIDARControlLEDStrip.h"
#include "ctre/Phoenix.h"
#include "Platform/Platform.h"

/* Destructor */
TaskLIDARControlLEDStrip::~TaskLIDARControlLEDStrip() { }

/* ILoopable */
void TaskLIDARControlLEDStrip::OnStart() {
}
void TaskLIDARControlLEDStrip::OnStop() {
}
bool TaskLIDARControlLEDStrip::IsDone() {
	return false;
}
void TaskLIDARControlLEDStrip::OnLoop() {
	float pulse = Tasks::taskMeasurePulseSensors->GetMeasuredPulseWidthsUs(
			CANifier::PWMChannel::PWMChannel3);

	/* Scale [0,8000] us to [0,360' Hue degrees */
	float hue = LinearInterpolation::Calculate(pulse, 0, 0, 8000, 360);

	Tasks::taskHSVControlLedStrip->Hue = hue;
	Tasks::taskHSVControlLedStrip->Saturation = 1;
	Tasks::taskHSVControlLedStrip->Value = 0.05; /* hard-code the brightness */
}
