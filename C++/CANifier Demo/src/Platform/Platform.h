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
	static CTRE::Tasking::Schedulers::ConcurrentScheduler* PeriodicTasks;
};

class Subsystems{
public:
	/* none */
};

class Tasks{
public:
	static TaskAnimateLEDStrip* taskAnimateLEDStrip;
	static TaskDirectControlLEDStrip* taskDirectControlArm;
	static TaskPWMmotorController* taskPWMmotorController;
	static TaskMeasurePulseSensors* taskMeasurePulseSensors;
	static TaskLIDARControlLEDStrip* taskLIDARControlLEDStrip;
	static TaskHSV* taskHSVControlLedStrip;

	static TaskMainLoop* taskMainLoop;

	static std::vector<CTRE::Tasking::ILoopable*> FullList;
};
