#pragma once

#include "ctre/phoenix/Tasking/ILoopable.h"
#include "ctre/phoenix/CANifier.h"

class TaskMeasurePulseSensors: public ctre::phoenix::tasking::ILoopable {
public:
	double _dutyCycleAndPeriods[4][2] =
	{
			{ 0, 0 }, /* PWM 0 */
			{ 0, 0 }, /* PWM 1 */
			{ 0, 0 }, /* PWM 2 */
			{ 0, 0 }, /* PWM 3 */
	};

	float GetMeasuredPulseWidthsUs(ctre::phoenix::CANifier::PWMChannel pwmCh);
	/* Destructor */
	virtual ~TaskMeasurePulseSensors();
	/* ILoopable */
	void OnStart();
	void OnStop();
	bool IsDone();
	void OnLoop();
};
