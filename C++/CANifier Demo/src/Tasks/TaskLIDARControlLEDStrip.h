#pragma once

#include "ctre/phoenix/Tasking/ILoopable.h"

class TaskLIDARControlLEDStrip : public CTRE::Tasking::ILoopable{
public:
	virtual ~TaskLIDARControlLEDStrip();
	/* ILoopable */
	void OnStart();
	void OnStop();
	bool IsDone();
	void OnLoop();
};
