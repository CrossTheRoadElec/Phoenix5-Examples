#include "Tasks/TaskTeleopDriveWithGamepad.h"
#include "Platform/Hardware.h"
#include "Platform/Tasks.h"

TaskTeleopDriveWithGamepad::~TaskTeleopDriveWithGamepad() {}
/* ILoopable */
void TaskTeleopDriveWithGamepad::OnStart() {
}
void TaskTeleopDriveWithGamepad::OnStop() {
}
bool TaskTeleopDriveWithGamepad::IsDone() {
	return false;
}
void TaskTeleopDriveWithGamepad::OnLoop() {
	float x = Hardware::gamepad->GetRawAxis(0);
	float y = -1 * Hardware::gamepad->GetRawAxis(1);
	float turn = Hardware::gamepad->GetRawAxis(2);

	CTRE::Utilities::Deadband(x, 0.15);
	CTRE::Utilities::Deadband(y, 0.15);
	CTRE::Utilities::Deadband(turn, 0.15);

	if (Tasks::LowBatteryDetect->GetBatteryIsLow()) {
		x *= 0.25;
		y *= 0.25;
		turn *= 0.25;
	}
	Hardware::drivetrain->Set(CTRE::Drive::Styles::Basic::PercentOutputBasic, y,
			x, turn);
}
