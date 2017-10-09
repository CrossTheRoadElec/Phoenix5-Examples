#pragma once

#include "ctre/phoenix/Tasking/ILoopable.h"
#include "ctre/phoenix/CANifier.h"

class TaskMeasurePulseSensors : public CTRE::Tasking::ILoopable{
public:
	float _dutyCycleAndPeriods[4][2] =
	{
			{ 0, 0 },
			{ 0, 0 },
			{ 0, 0 },
			{ 0, 0 },
	};

	float GetMeasuredPulseWidthsUs(CTRE::CANifier::PWMChannel pwmCh);
	virtual ~TaskMeasurePulseSensors();
	/* ILoopable */
	void OnStart();
	void OnStop();
	bool IsDone();
	void OnLoop();
};
