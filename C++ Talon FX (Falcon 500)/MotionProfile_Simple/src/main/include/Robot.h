#pragma once

#include <frc/TimedRobot.h>
#include <frc/Joystick.h>

#include "ctre/Phoenix.h"
#include "MotionProfileConfiguration.h"
#include "PlotThread.h"

class Robot : public frc::TimedRobot
{
public:
	void SimulationInit() override;
	void SimulationPeriodic() override;

	void RobotInit() override;

	void AutonomousInit() override;
	void AutonomousPeriodic() override;

	void TeleopInit() override;
	void TeleopPeriodic() override;

	void TestInit() override;
	void TestPeriodic() override;

private:
	int _state;
	WPI_TalonFX _master{1};
	frc::Joystick _joy{0};
	BufferedTrajectoryPointStream _bufferedStream;
	MotionProfileConfiguration _configuration;
	PlotThread _plotThread{&_master};

	void InitBuffer(const double profile[][3], int totalCnt);
};
