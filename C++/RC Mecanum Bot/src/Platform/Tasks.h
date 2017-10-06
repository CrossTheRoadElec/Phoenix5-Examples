#pragma once

#include "Tasks/TaskEnableRobot.h"
#include "Tasks/TaskLEDStrip.h"
#include "Tasks/TaskLowBatteryDetect.h"
#include "Tasks/TaskTeleopDriveWithGamepad.h"
#include "Tasks/TaskTeleopDriveWithRC.h"

class Tasks{
public:
    /* Subsystem tasks.  A task could be all the functionality of a subsystem like a robot arm,
     * or could be an individual action apply to a subsystem, such as up reading a game-pad and applying it to the drive-train.
     * Use 'public static' because these are single objects.*/
	static TaskEnableRobot* EnableRobot;
	static TaskLEDStrip* LEDStrip;
	static TaskTeleopDriveWithRC* TeleopDriveWithRC;
	static TaskTeleopDriveWithGamepad* TeleopDriveWithGamepad;
	static TaskLowBatteryDetect* LowBatteryDetect;
};
