#pragma once

#include <frc/TimedRobot.h>
#include <frc/Joystick.h>
#include "ctre/Phoenix.h"
#include "PlotThread.h"
#include "MasterProfileConfiguration.h"
#include "FollowerProfileConfiguration.h"

class Robot : public frc::TimedRobot
{
public:
	void RobotInit() override;

	void AutonomousInit() override;
	void AutonomousPeriodic() override;

	void TeleopInit() override;
	void TeleopPeriodic() override;

	void TestInit() override;
	void TestPeriodic() override;

private:
	int _state;

	TalonSRX *_rightMaster;
	TalonSRX *_leftMaster;

	MasterProfileConfiguration *_masterConfig;
	FollowerProfileConfiguration *_followConfig;

	PigeonIMU *_pidgey;

	frc::Joystick *_joystick;

	BufferedTrajectoryPointStream *_bufferedStream;
	PlotThread *_plotThread;

	void InitBuffer(const double profile[][3], int totalCnt, double rotations);
};
