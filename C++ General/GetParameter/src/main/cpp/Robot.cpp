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

class Robot: public TimedRobot {
public:
	//Create Talon
	TalonSRX * _talon;

	void RobotInit() {
		//Initialize talon with ID 3
		_talon = new WPI_TalonSRX(3);
		/* Factory Default all hardware to prevent unexpected behaviour */
		_talon->ConfigFactoryDefault();

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

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
