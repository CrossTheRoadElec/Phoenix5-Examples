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

	/* TalonSRX */
	TalonSRX * _tal;

	/* CANifier */
	CANifier * _can;

	/* Joystick */
	Joystick * _joy;

	/* Variable to keep track of loop count */
	unsigned int count;


	void RobotInit() {
		/* Instantiate objects */
		_can = new CANifier(0);

		_tal = new TalonSRX(4);

		_joy = new Joystick(0);
	}

	void TeleopInit() {
		/* Initialize count to 0 */
		count = 0;

		/* Configure sensor on talon to check CANifier */
		_tal->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);

		/* Set sensor positions to some known position */
		_tal->SetSelectedSensorPosition(33, 0, 0);
		_can->SetQuadraturePosition(33, 0);

		/* Configure velocity measurements to what we want */
		_can->ConfigVelocityMeasurementPeriod(CANifierVelocityMeasPeriod::Period_100Ms, 10);
		_can->ConfigVelocityMeasurementWindow(64, 10);
	}

	void TeleopPeriodic() {
		/* Every 20th loop print */
		if(count++ >= 20)
		{
			/* CANifier */
			std::cout << "CANifier:\tPosition: " << _can->GetQuadraturePosition() << "\tVelocity" << _can->GetQuadratureVelocity() <<

			/* TalonSRX */
			std::endl << "Talon:\t\t\tPosition: " << _tal->GetSelectedSensorPosition(0) <<"\tVelocity" << _tal->GetSelectedSensorVelocity(0)

			/* New line to deliniate each loop */
			<< std::endl << std::endl;

			/* Reset count */
			count = 0;
		}

		/* Run talon in PercentOutput mode always */
		_tal->Set(ControlMode::PercentOutput, _joy->GetY());
	}

	void TestPeriodic() {}

};

START_ROBOT_CLASS(Robot)
