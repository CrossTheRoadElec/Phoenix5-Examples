#pragma once

#include "ctre/phoenix/Tasking/ILoopable.h"

class TaskLIDARControlLEDStrip: public ctre::phoenix::tasking::ILoopable {
public:
	/* Destructor */
	virtual ~TaskLIDARControlLEDStrip();
	/* ILoopable */
	void OnStart();
	void OnStop();
	bool IsDone();
	void OnLoop();
};
