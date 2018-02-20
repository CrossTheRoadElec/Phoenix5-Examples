/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team217.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {

	TalonSRX _tal = new TalonSRX(0);
	Joystick _joy = new Joystick(0);

	boolean _currentLimEn = true;
	boolean _btn5 = false;

	public void teleopInit() {
		/* factory default your Talon before running this,
		 * This way we can just call the config* routines that are
		 * different than default values.*/

		/* start current limit at 15 Amps, then hold at 10 Amps */
		final int kPeakCurrentAmps = 15; /* threshold to trigger current limit */
		final int kPeakTimeMs = 0; /* how long after Peak current to trigger current limit */
		final int kContinCurrentAmps = 10; /* hold current after limit is triggered */

		_tal.configPeakCurrentLimit(kPeakCurrentAmps, 10);
		_tal.configPeakCurrentDuration(kPeakTimeMs, 10); /* this is a necessary call to avoid errata. */
		_tal.configContinuousCurrentLimit(kContinCurrentAmps, 10);
		_tal.enableCurrentLimit(_currentLimEn); /* honor initial setting */

		/* setup a basic closed loop */
		_tal.setNeutralMode(NeutralMode.Brake);
		_tal.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
		_tal.setSensorPhase(true); /* flip until sensor is in phase, or closed-loop will not work */
		_tal.config_kP(0, 2.0, 10);
		_tal.config_kI(0, 0.0, 10);
		_tal.config_kD(0, 0.0, 10);
		_tal.config_kF(0, 0.0, 10);
	}

	public void teleopPeriodic() {
		/* get inputs */
		boolean btn1 = _joy.getRawButton(1);
		boolean btn2 = _joy.getRawButton(2);
		boolean btn5 = _joy.getRawButton(5);
		double stick = -1.0 * _joy.getY();

		if (btn1) {
			/* on button 1 press, manual control with stick */
			_tal.set(ControlMode.PercentOutput, stick);
		} else if (btn2) {
			/* on button 2 press, servo back to zero */
			double targetPos = 0;
			_tal.set(ControlMode.Position, targetPos);
		} else {
			/* otherwise stop output */
			_tal.set(ControlMode.PercentOutput, 0);
		}

		/* on button5 (shoulder button on Logitech gamepad) */
		if (!_btn5 && btn5) {
			/* toggle current limit */
			_currentLimEn = !_currentLimEn;
			/* update Talon current limit */
			_tal.enableCurrentLimit(_currentLimEn);
			/* print to DS */
			System.out.println("EnableCurrentLimit:"  + _currentLimEn);
		}
		/* save button state for next loop */
		_btn5 = btn5;
	}

}
