#pragma once

#include "WPILIB.h"
#include "ctre/phoenix/Tasking/Schedulers/ConcurrentScheduler.h"
#include "ctre/phoenix/Tasking/ILoopable.h"
#include "ctre/phoenix/CANifier.h"
#include "ctre/phoenix/core/ctre.h"
#include <vector>

/* Include all tasks headers */
#include "Tasks/TaskAnimateLEDStrip.h"
#include "Tasks/TaskDirectControlLEDStrip.h"
#include "Tasks/TaskHSV.h"
#include "Tasks/TaskLIDARControlLEDStrip.h"
#include "Tasks/TaskMainLoop.h"
#include "Tasks/TaskMeasurePulseSensors.h"
#include "Tasks/TaskPWMmotorController.h"

class Constants{
public:
	const float GamepadDeadband = 0.03;

	static const int Gamepad_x = 0;
	static const int Gamepad_y = 1;
	static const int GamePadAxis_red = 0;
	static const int GamePadAxis_green = 1;
	static const int GamePadAxis_blue = 5;

	static const CTRE::CANifier::PWMChannel MotorControllerCh = CTRE::CANifier::PWMChannel::PWMChannel2;
};

class Hardware{
public:
	static CTRE::CANifier* canifier;
	static frc::Joystick* gamepad;
};

class Schedulers{
public:
    /* the schedulers.  Minimally you will likely want one periodic scheduler to run the normal tasks.
     * Additional schedulers could be ConsecutiveSchedulers for entire autonomous movements or pre-configured maneuvers.
     * Use 'public static' because these are single objects. */
	static CTRE::Tasking::Schedulers::ConcurrentScheduler* PeriodicTasks;
};

class Tasks{
public:
    /* Subsystem tasks.  A task could be all the functionality of a subsystem like a robot arm,
     * or could be an individual action to a subsystem, such as up reading a game-pad and applying it to the drive-train.
     * Use 'public static' because these are single objects. */
	static TaskAnimateLEDStrip* taskAnimateLEDStrip;
	static TaskDirectControlLEDStrip* taskDirectControlArm;
	static TaskPWMmotorController* taskPWMmotorController;
	static TaskMeasurePulseSensors* taskMeasurePulseSensors;
	static TaskLIDARControlLEDStrip* taskLIDARControlLEDStrip;
	static TaskHSV* taskHSVControlLedStrip;

	static TaskMainLoop* taskMainLoop;

    /* Insert all Tasks below in the Full List so they get auto inserted, see Robot.java to see how this works.*/
	static std::vector<CTRE::Tasking::ILoopable*> FullList;
};
