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

class Robot: public frc::IterativeRobot {
public:

	//Create Talon
	TalonSRX * _talon;

	void RobotInit() {
		//Initialize talon with ID 3
		_talon = new TalonSRX(3);

		//Configure parameters to talon to get later
		_talon->Config_kP(0, 0.2, 10);
		_talon->ConfigForwardSoftLimitThreshold(200, 10);
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
