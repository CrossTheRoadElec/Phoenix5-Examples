#pragma once

#include "ctre/phoenix/Tasking/ILoopable.h"
#include "ctre/Phoenix.h"

class TaskHSV: public ctre::phoenix::tasking::ILoopable {
public:
	float Hue;
	float Saturation;
	float Value;
	
	/* Destructor */
	virtual ~TaskHSV();

	/* ILoopable */
	void OnStart();
	void OnStop();
	bool IsDone();
	void OnLoop();
private:
	float _r, _g, _b;
	
	MovingAverage* _averageR = new MovingAverage(10);
	MovingAverage* _averageG = new MovingAverage(10);
	MovingAverage* _averageB = new MovingAverage(10);
};
