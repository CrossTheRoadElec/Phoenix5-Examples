#include "Tasks/TaskTeleopDriveWithRC.h"
#include "Platform/Hardware.h"
#include "Platform/Tasks.h"

TaskTeleopDriveWithRC::~TaskTeleopDriveWithRC() {}
/* ILoopable */
void TaskTeleopDriveWithRC::OnStart() {
}
void TaskTeleopDriveWithRC::OnStop() {
}
bool TaskTeleopDriveWithRC::IsDone() {
	return false;
}
void TaskTeleopDriveWithRC::OnLoop() {
	/* PID gains */
	Hardware::HoldHeadingParameters->ServoParameters::P = 0.01;
	Hardware::HoldHeadingParameters->ServoParameters::D = 0.001;

	/* poll RC Radio */
	float forward = Hardware::Futuba3Ch->GetDutyCyclePerc(
			CTRE::RCRadio3Ch::Channel::Channel2);
	float turn = Hardware::Futuba3Ch->GetDutyCyclePerc(
			CTRE::RCRadio3Ch::Channel::Channel1);
	float strafe = 0;

	bool toggleSwitch = Hardware::Futuba3Ch->GetSwitchValue(CTRE::RCRadio3Ch::Channel::Channel3);

	CTRE::Utilities::Deadband(forward, .15);
	CTRE::Utilities::Deadband(turn, .15);
	CTRE::Utilities::Deadband(strafe, .15);

	if (Tasks::LowBatteryDetect->GetBatteryIsLow()) {
		forward *= 0.25;
		strafe *= 0.25;
		turn *= 0.25;
	}

	if(toggleSwitch && (turn == 0))
	{
		Hardware::HoldHeadingServo->Set(CTRE::Drive::Styles::Basic::PercentOutputBasic, forward, strafe);
	}else{
		Hardware::drivetrain->Set(CTRE::Drive::Styles::Basic::PercentOutputBasic,
				forward, strafe, turn);
	}
}
