#pragma once

#include "ctre/phoenix/Tasking/ILoopable.h"

class TaskAnimateLEDStrip : public CTRE::Tasking::ILoopable{
public:
	virtual ~TaskAnimateLEDStrip();
	/* ILoopable */
	void OnStart();
	void OnStop();
	bool IsDone();
	void OnLoop();
private:
	float _hue;
};
