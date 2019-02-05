/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

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
