#include "Tasks/TaskAnimateLEDStrip.h"
#include "Tasks/TaskHSV.h"
#include "Platform/Platform.h"

/* Destructor */
TaskAnimateLEDStrip::~TaskAnimateLEDStrip() { }

/* ILoopable */
void TaskAnimateLEDStrip::OnStart() {
}
void TaskAnimateLEDStrip::OnStop() {
}
bool TaskAnimateLEDStrip::IsDone() {
	return false;
}
void TaskAnimateLEDStrip::OnLoop() {
	/* Ramp through the outer rim of the HSV color wheel */
	_hue += 1;
	if (_hue >= 360) {
		_hue = 0;
	}

	/* update HSV target */
	Tasks::taskHSVControlLedStrip->Hue = _hue;
	Tasks::taskHSVControlLedStrip->Saturation = 1.0; /* outer rim of HSV color wheel */
	Tasks::taskHSVControlLedStrip->Value = 0.05; /* hard-code the brightness */
}
