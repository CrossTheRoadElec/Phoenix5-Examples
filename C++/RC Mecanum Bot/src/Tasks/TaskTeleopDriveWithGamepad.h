#pragma once

#include "ctrlib/Tasking/ILoopable.h"

class TaskTeleopDriveWithGamepad : public CTRE::Tasking::ILoopable {
public:
	virtual ~TaskTeleopDriveWithGamepad();
	/* ILoopable */
	void OnStart();
	void OnStop();
	bool IsDone();
	void OnLoop();
};
