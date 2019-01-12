#pragma once

#include "ctre/phoenix/Tasking/ILoopable.h"

class TaskAnimateLEDStrip: public ctre::phoenix::tasking::ILoopable {
public:
	/* Destructor */
	virtual ~TaskAnimateLEDStrip();
	/* ILoopable */
	void OnStart();
	void OnStop();
	bool IsDone();
	void OnLoop();
private:
	float _hue;
};
