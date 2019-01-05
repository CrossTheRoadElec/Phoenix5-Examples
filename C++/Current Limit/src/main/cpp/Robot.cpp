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
	TalonSRX * _tal = new TalonSRX(0);
	Joystick * _joy = new Joystick(0);

	bool _currentLimEn = true;
	bool _btn5 = false;

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

START_ROBOT_CLASS(Robot)
