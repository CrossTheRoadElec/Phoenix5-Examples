#pragma once

#include "ctre/phoenix/Tasking/ILoopable.h"

class TaskPWMmotorController: public ctre::phoenix::tasking::ILoopable {
public:
	float _percentOutput;
	bool _running;			//Track for TaskMainLoop

	/* Destructor */
	virtual ~TaskPWMmotorController();
	/* ILoopable */
	void OnStart();
	void OnStop();
	bool IsDone();
	void OnLoop();
};
