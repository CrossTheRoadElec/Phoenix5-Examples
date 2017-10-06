#include <Tasks/TaskLowBatteryDetect.h>
#include "Platform/Hardware.h"

TaskLowBatteryDetect::~TaskLowBatteryDetect(){}
bool TaskLowBatteryDetect::GetBatteryIsLow(){
	return BatteryIsLow;
}
void TaskLowBatteryDetect::SetBatteryIsLow(bool state){
	BatteryIsLow = state;
}
/* ILoopable */
void TaskLowBatteryDetect::OnStart() {
}
void TaskLowBatteryDetect::OnStop() {
}
bool TaskLowBatteryDetect::IsDone() {
	return false;
}
void TaskLowBatteryDetect::OnLoop() {
	float vbat;

	/* get the average voltage from a couple talons */
	vbat = 0;
	vbat += Hardware::leftFront->GetBusVoltage();
	vbat += Hardware::rightFront->GetBusVoltage();
	vbat *= 0.5f;

	if (vbat > 10.50) {
		_downCount = 0;
		if (_upCount < 100)
			++_upCount;
	} else if (vbat < 10.00) {
		_upCount = 0;
		if (_downCount < 100)
			++_downCount;
	}

	if (_downCount > 50) {
		BatteryIsLow = true;
	} else if (_upCount > 50) {
		BatteryIsLow = false;
	} else {
		//don't change filter ouput
	}
}
