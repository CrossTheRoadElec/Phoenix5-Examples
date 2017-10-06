#pragma once

#include "ctrlib/Tasking/ILoopable.h"

class TaskTeleopDriveWithRC : public CTRE::Tasking::ILoopable{
public:
	virtual ~TaskTeleopDriveWithRC();
	/* ILoopable */
	void OnStart();
	void OnStop();
	bool IsDone();
	void OnLoop();
private:
	float _pulseWidUs[4];
	float _peridoUs[4];
	bool _holdHeading = false;
};
