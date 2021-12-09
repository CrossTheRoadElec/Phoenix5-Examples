#pragma once

#include <frc/TimedRobot.h>
#include <frc/Joystick.h>
#include "ctre/Phoenix.h"
#include "DrivebaseSimFX.h"
#include "PlotThread.h"
#include "MasterProfileConfiguration.h"
#include "FollowerProfileConfiguration.h"

class Robot : public frc::TimedRobot
{
public:
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

	WPI_TalonFX _rightMaster{1};
	WPI_TalonFX _leftMaster{2};

	WPI_PigeonIMU _pidgey{3};

	frc::Joystick _joystick{0};

	MasterProfileConfiguration _masterConfig{_leftMaster, _pidgey};
	FollowerProfileConfiguration _followConfig;

	BufferedTrajectoryPointStream _bufferedStream;
	PlotThread _plotThread{&_rightMaster};

	void InitBuffer(const double profile[][3], int totalCnt, double rotations);

	DrivebaseSimFX _driveSim{_leftMaster, _rightMaster, _pidgey};
};
