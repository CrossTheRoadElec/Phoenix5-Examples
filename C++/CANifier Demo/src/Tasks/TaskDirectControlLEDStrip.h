#pragma once

#include "ctre/phoenix/Tasking/ILoopable.h"

class TaskDirectControlLEDStrip: public ctre::phoenix::tasking::ILoopable {
public:
	virtual ~TaskDirectControlLEDStrip();
	/* ILoopable */
	void OnStart();
	void OnStop();
	bool IsDone();
	void OnLoop();
};
