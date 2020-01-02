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

	TalonSRX * _magTalon;
	TalonSRX * _tachTalon;
	Joystick * _joy;

	void RobotInit() {
		_magTalon = new TalonSRX(4);
		_tachTalon = new TalonSRX(5);
		_joy = new Joystick(0);

		/* Factory Default all hardware to prevent unexpected behaviour */
		_magTalon->ConfigFactoryDefault();
		_tachTalon->ConfigFactoryDefault();
	}

	void TeleopInit() {
		
	    /* nonzero to block the config until success, zero to skip checking */
    	const int kTimeoutMs = 30;

        //Configure talon to read magencoder values
		_magTalon->ConfigSelectedFeedbackSensor(
				FeedbackDevice::CTRE_MagEncoder_Relative, 0, kTimeoutMs);

		//Configure talon to read tachometer values
		_tachTalon->ConfigSelectedFeedbackSensor(
				FeedbackDevice::Tachometer, 0, kTimeoutMs);

		/* read section 7.9 Tachometer Measurement in software reference manual */
		//Edges per cycle = 2 (WHITE black WHITE black per revolution)
		int edgesPerCycle = 2;
		_tachTalon->ConfigSetParameter((ParamEnum) 430, edgesPerCycle, 0, 0, kTimeoutMs);
		// additional filtering if need be.
		int filterWindowSize = 1;
		_tachTalon->ConfigSetParameter((ParamEnum) 431, filterWindowSize, 0, 0, kTimeoutMs);
	}

	void TeleopPeriodic() {
		_magTalon->Set(ControlMode::PercentOutput, _joy->GetY());
		/* get the velocities of two talons,
		 * one uses quadrature (mag encoder), the other uses Talon-Tach */
		double magVel_UnitsPer100ms = _magTalon->GetSelectedSensorVelocity(0);
		double tachVel_UnitsPer100ms = _tachTalon->GetSelectedSensorVelocity(0);

		/* convert to RPM */
		// https://github.com/CrossTheRoadElec/Phoenix-Documentation#what-are-the-units-of-my-sensor
		//MagRPM = magVel [units/kT] * 600 [kTs/minute] / 4096(units/rev), where kT = 100ms
		double magRPM = magVel_UnitsPer100ms * 600 / 4096;
		//TachRPM = tachVel [units/kT] * 600 [kTs/minute] / 1024(units/rev), where kT = 100ms
		double tachRPM = tachVel_UnitsPer100ms * 600 / 1024;

		//Write to DS
		std::cout 	<< "Mag encoder speed is: " << magRPM << "\t"
					<< "Tach speed is: " << tachRPM << std::endl;
	}

	void TestPeriodic() {
	}

private:
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
