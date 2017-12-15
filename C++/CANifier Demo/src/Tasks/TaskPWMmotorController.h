#pragma once

#include "ctre/phoenix/Tasking/ILoopable.h"

class TaskPWMmotorController : public CTRE::Tasking::ILoopable{
public:
	float _percentOutput;
	bool _running;			//Track for TaskMainLoop

	virtual ~TaskPWMmotorController();
	/* ILoopable */
	void OnStart();
	void OnStop();
	bool IsDone();
	void OnLoop();
};
