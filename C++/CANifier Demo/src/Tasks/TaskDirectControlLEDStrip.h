#pragma once

#include "ctre/phoenix/Tasking/ILoopable.h"

class TaskDirectControlLEDStrip : public CTRE::Tasking::ILoopable{
public:
	virtual ~TaskDirectControlLEDStrip();
	/* ILoopable */
	void OnStart();
	void OnStop();
	bool IsDone();
	void OnLoop();
};
