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
#include "PhysicsSim.h"

using namespace frc;

class Robot: public TimedRobot {
public:
	TalonSRX * _tal = new WPI_TalonSRX(0);
	Joystick * _joy = new Joystick(0);

	bool _currentLimEn = true;
	bool _btn5 = false;

	void SimulationInit() {
		PhysicsSim::GetInstance().AddTalonSRX(*_tal, 0.75, 2000);
	}
	void SimulationPeriodic() {
		PhysicsSim::GetInstance().Run();
	}
	
	void RobotInit(){
		/* Factory Default all hardware to prevent unexpected behaviour */
		_tal->ConfigFactoryDefault();
	}
	void TeleopInit() {
		/* factory default your Talon before running this,
		 * This way we can just call the config* routines that are
		 * different than default values.*/

		/* start current limit at 15 Amps, then hold at 10 Amps */
		const int kPeakCurrentAmps = 15; /* threshold to trigger current limit */
		const int kPeakTimeMs = 0; /* how long after Peak current to trigger current limit */
		const int kContinCurrentAmps = 10; /* hold current after limit is triggered */
	    /* nonzero to block the config until success, zero to skip checking */
    	const int kTimeoutMs = 30;

		_tal->ConfigPeakCurrentLimit(kPeakCurrentAmps, kTimeoutMs);
		_tal->ConfigPeakCurrentDuration(kPeakTimeMs, kTimeoutMs); /* this is a necessary call to avoid errata. */
		_tal->ConfigContinuousCurrentLimit(kContinCurrentAmps, kTimeoutMs);
		_tal->EnableCurrentLimit(_currentLimEn); /* honor initial setting */

		/* setup a basic closed loop */
		_tal->SetNeutralMode(NeutralMode::Brake);
		_tal->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, kTimeoutMs);
		_tal->SetSensorPhase(true); /* flip until sensor is in phase, or closed-loop will not work */
		_tal->Config_kP(0, 2.0, 10);
		_tal->Config_kI(0, 0.0, 10);
		_tal->Config_kD(0, 0.0, 10);
		_tal->Config_kF(0, 0.0, 10);
	}

	void TeleopPeriodic() {
		/* get inputs */
		bool btn1 = _joy->GetRawButton(1);
		bool btn2 = _joy->GetRawButton(2);
		bool btn5 = _joy->GetRawButton(5);
		double stick = -1.0 * _joy->GetY();

		if (btn1) {
			/* on button 1 press, manual control with stick */
			_tal->Set(ControlMode::PercentOutput, stick);
		} else if (btn2) {
			/* on button 2 press, servo back to zero */
			double targetPos = 0;
			_tal->Set(ControlMode::Position, targetPos);
		} else {
			/* otherwise stop output */
			_tal->Set(ControlMode::PercentOutput, 0);
		}
		std::cout << _tal->GetMotorOutputPercent() << std::endl;

		/* on button5 (shoulder button on Logitech gamepad) */
		if (!_btn5 && btn5) {
			/* toggle current limit */
			_currentLimEn = !_currentLimEn;
			/* update Talon current limit */
			_tal->EnableCurrentLimit(_currentLimEn);
			/* print to DS */
			printf("EnableCurrentLimit:%i\n", (int) _currentLimEn);
		}
		/* save button state for next loop */
		_btn5 = btn5;
	}

private:
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif