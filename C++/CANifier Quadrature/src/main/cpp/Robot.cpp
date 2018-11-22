/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <iostream>
#include <string>

#include "frc/WPILib.h"
#include "ctre/Phoenix.h"

using namespace frc;

class Robot : public TimedRobot {
public:
	/* Hardware */
	TalonSRX * _tal;
	CANifier * _can;
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
        
	    /* nonzero to block the config until success, zero to skip checking */
	    const int kTimeoutMs = 30;
		
        /* Configure sensor on talon to check CANifier */
		_tal->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, kTimeoutMs);

		/* Set sensor positions to some known position */
		_tal->SetSelectedSensorPosition(33, 0, kTimeoutMs);
		_can->SetQuadraturePosition(33, kTimeoutMs);

		/* Configure velocity measurements to what we want */
		_can->ConfigVelocityMeasurementPeriod(CANifierVelocityMeasPeriod::Period_100Ms, kTimeoutMs);
		_can->ConfigVelocityMeasurementWindow(64, kTimeoutMs);
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
};

START_ROBOT_CLASS(Robot)
