#pragma once

#include <frc/TimedRobot.h>
#include <frc/Joystick.h>
#include "ctre/Phoenix.h"
#include "DrivebaseSimSRX.h"
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

	WPI_TalonSRX _rightMaster{1};
	WPI_TalonSRX _leftMaster{2};

	WPI_PigeonIMU _pidgey{3};

	MasterProfileConfiguration _masterConfig{_leftMaster, _pidgey};
	FollowerProfileConfiguration _followConfig;

	frc::Joystick _joystick{0};

	BufferedTrajectoryPointStream _bufferedStream;
	PlotThread _plotThread{&_rightMaster};

	DrivebaseSimSRX _driveSim{_leftMaster, _rightMaster, _pidgey};

	void InitBuffer(const double profile[][3], int totalCnt, double rotations);
};
