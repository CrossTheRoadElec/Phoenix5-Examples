#pragma once

#include "ctrlib/Tasking/ILoopable.h"
#include "ctrlib/HsvToRgb.h"

class TaskLEDStrip : public CTRE::Tasking::ILoopable{
public:
	virtual ~TaskLEDStrip();
	/* ILoopable */
	void OnStart();
	void OnStop();
	bool IsDone();
	void OnLoop();
private:
	float _theta;
	float _saturation;
	float _value = 0.05;
};
