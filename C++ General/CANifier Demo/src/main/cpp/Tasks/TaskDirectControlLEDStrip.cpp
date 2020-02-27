#include <math.h>

#include "Tasks/TaskDirectControlLEDStrip.h"
#include "ctre/Phoenix.h"
#include "Platform/Platform.h"
#include "ctre/phoenix/Utilities.h"

#define PI 3.14159265358979

/* Destructor */
TaskDirectControlLEDStrip::~TaskDirectControlLEDStrip() { }

/* ILoopable */
void TaskDirectControlLEDStrip::OnStart() {
}
void TaskDirectControlLEDStrip::OnStop() {
}
bool TaskDirectControlLEDStrip::IsDone() {
	return false;
}
void TaskDirectControlLEDStrip::OnLoop() {
	/* Get an x and y pair */
	float x = Hardware::gamepad->GetRawAxis(0);
	float y = Hardware::gamepad->GetRawAxis(1);

	/* Calculate theta in degrees*/
	float theta = (float) atan2(x, y) * 180 / PI;
	/* Cap the magnitude to '1'. This will be our saturation(How far away from white) */
	float saturation = (float) sqrt(x * x + y * y);
	saturation = Utilities::cap(saturation, 1);
	/* Select a value of '1', how afar away from black */
	Tasks::taskHSVControlLedStrip->Hue = theta;
	Tasks::taskHSVControlLedStrip->Saturation = saturation;
	Tasks::taskHSVControlLedStrip->Value = 1; /* scale down for brightness */
}
