#pragma once

#include "ctrlib/Tasking/ILoopable.h"

class TaskLowBatteryDetect : public CTRE::Tasking::ILoopable{
public:
	virtual ~TaskLowBatteryDetect();
	bool GetBatteryIsLow();
	/* ILoopable */
	void OnStart();
	void OnStop();
	bool IsDone();
	void OnLoop();
private:
	int _downCount = 0;
	int _upCount = 0;
	bool BatteryIsLow;
	void SetBatteryIsLow(bool state);
};
