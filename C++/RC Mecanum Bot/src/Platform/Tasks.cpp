#include "Platform/Tasks.h"

TaskEnableRobot* Tasks::EnableRobot = new TaskEnableRobot();
TaskLEDStrip* Tasks::LEDStrip = new TaskLEDStrip();
TaskTeleopDriveWithRC* Tasks::TeleopDriveWithRC = new TaskTeleopDriveWithRC();
TaskTeleopDriveWithGamepad* Tasks::TeleopDriveWithGamepad = new TaskTeleopDriveWithGamepad();
TaskLowBatteryDetect* Tasks::LowBatteryDetect = new TaskLowBatteryDetect();
