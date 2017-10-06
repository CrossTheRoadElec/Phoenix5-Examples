#pragma once

#include "Tasks/TaskTeleopDriveWithGamepad.h"
#include "Tasks/TaskTeleopDriveWithRC.h"
#include "ctrlib/Tasking/ILoopable.h"

class TaskEnableRobot : public CTRE::Tasking::ILoopable {
public:
	virtual ~TaskEnableRobot();
	/* ILoopable */
	void OnStart();
	void OnStop();
	bool IsDone();
	void OnLoop();
};
