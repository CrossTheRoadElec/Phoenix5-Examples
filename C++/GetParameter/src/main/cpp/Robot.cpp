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

class Robot: public TimedRobot {
public:
	//Create Talon
	TalonSRX * _talon;

	void RobotInit() {
		//Initialize talon with ID 3
		_talon = new TalonSRX(3);

	    /* nonzero to block the config until success, zero to skip checking */
    	const int kTimeoutMs = 30;
		//Configure parameters to talon to get later
		_talon->Config_kP(0, 0.2, kTimeoutMs);
		_talon->ConfigForwardSoftLimitThreshold(200, kTimeoutMs);
	}

	void TeleopInit() {
		//Get Parameters from Talon
		std::cout << "Talon kP is: "
				<< _talon->ConfigGetParameter(ParamEnum::eProfileParamSlot_P, 0,
						0) << std::endl << "Talon Forward Soft Limit is: "
				<< _talon->ConfigGetParameter(
						ParamEnum::eForwardSoftLimitThreshold, 0, 0)
				<< std::endl;
	}
};

START_ROBOT_CLASS(Robot)
