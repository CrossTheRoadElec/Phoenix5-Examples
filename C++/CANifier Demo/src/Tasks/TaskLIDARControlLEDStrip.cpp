#include <Tasks/TaskLIDARControlLEDStrip.h>
#include "ctre/phoenix/LinearInterpolation.h"
#include "Platform/Platform.h"

TaskLIDARControlLEDStrip::~TaskLIDARControlLEDStrip() {}
/* ILoopable */
void TaskLIDARControlLEDStrip::OnStart() {
}
void TaskLIDARControlLEDStrip::OnStop() {
}
bool TaskLIDARControlLEDStrip::IsDone() {
	return false;
}
void TaskLIDARControlLEDStrip::OnLoop(){
    float pulse = Tasks::taskMeasurePulseSensors->GetMeasuredPulseWidthsUs(CTRE::CANifier::PWMChannel::PWMChannel3);

    /* scale [0,8000] us to [0,360' Hue Deg */
    float hue = CTRE::LinearInterpolation::Calculate(pulse, 0, 0, 8000, 360);

    Tasks::taskHSVControlLedStrip->Hue = hue;
    Tasks::taskHSVControlLedStrip->Saturation = 1;
    Tasks::taskHSVControlLedStrip->Value = 0.05; /* hard-code the brightness */
}
