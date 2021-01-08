#pragma once

#include <frc/TimedRobot.h>
#include <frc/Joystick.h>

#include "ctre/Phoenix.h"
#include "MotionProfileConfiguration.h"
#include "PlotThread.h"

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

	void SimulationInit() override;
	void SimulationPeriodic() override;

private:
	int _state;
	TalonSRX *_master;
	frc::Joystick *_joy;
	BufferedTrajectoryPointStream *_bufferedStream;
	MotionProfileConfiguration *_configuration;
	PlotThread *_plotThread;

	void InitBuffer(const double profile[][3], int totalCnt);
};
