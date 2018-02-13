/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <iostream>
#include <string>

#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include "ctre/Phoenix.h"
#include "WPILib.h"

class Robot : public frc::IterativeRobot {
public:

	TalonSRX * _tal;
	CANifier * _can;
	Joystick * _joy;

	unsigned int count;
	void RobotInit() {

		_can = new CANifier(0);

		_tal = new TalonSRX(4);

		_joy = new Joystick(0);
	}

	void TeleopInit() {
		count = 0;

		_tal->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);

		_tal->SetSelectedSensorPosition(-8388000, 0, 0);
		_can->SetQuadraturePosition(-8388000, 0);

		_can->ConfigVelocityMeasurementPeriod(CANifierVelocityMeasPeriod::Period_100Ms, 10);
		_can->ConfigVelocityMeasurementWindow(1, 10);
	}

	void TeleopPeriodic() {

		if(count++ >= 20)
		{
			std::cout << "CANifier:\tPosition: " << _can->GetQuadraturePosition() << "\tVelocity" << _can->GetQuadratureVelocity() <<
			std::endl << "Talon:\t\t\tPosition: " << _tal->GetSelectedSensorPosition(0) <<"\tVelocity" << _tal->GetSelectedSensorVelocity(0)
					<< std::endl << std::endl;
			count = 0;
		}
		_tal->Set(ControlMode::PercentOutput, _joy->GetY());
	}

	void TestPeriodic() {}

};

START_ROBOT_CLASS(Robot)
