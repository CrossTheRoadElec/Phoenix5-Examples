/**
 * Phoenix Software License Agreement
 *
 * Copyright (C) Cross The Road Electronics.  All rights
 * reserved.
 * 
 * Cross The Road Electronics (CTRE) licenses to you the right to 
 * use, publish, and distribute copies of CRF (Cross The Road) firmware files (*.crf) and 
 * Phoenix Software API Libraries ONLY when in use with CTR Electronics hardware products
 * as well as the FRC roboRIO when in use in FRC Competition.
 * 
 * THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT
 * WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT
 * LIMITATION, ANY WARRANTY OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * CROSS THE ROAD ELECTRONICS BE LIABLE FOR ANY INCIDENTAL, SPECIAL, 
 * INDIRECT OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF
 * PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
 * BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE
 * THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER
 * SIMILAR COSTS, WHETHER ASSERTED ON THE BASIS OF CONTRACT, TORT
 * (INCLUDING NEGLIGENCE), BREACH OF WARRANTY, OR OTHERWISE
 */
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
		/* Factory Default all hardware to prevent unexpected behaviour */
		_can->ConfigFactoryDefault();
		_tal->ConfigFactoryDefault();

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

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif