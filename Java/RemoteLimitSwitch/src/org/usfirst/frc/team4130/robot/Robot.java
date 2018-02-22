/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4130.robot;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.RemoteLimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends IterativeRobot {

	/* hardware objects - use references instead of pointers to match Java examples. */
	TalonSRX _motorCntrller = new TalonSRX(2); // could also be Victor SPX if using remote sensor features.

	CANifier _canifLimits = new CANifier(2); /* use this CANifier for limit switches */
	TalonSRX _talonLimits = new TalonSRX(5); /* use this Talon for limit switches */

	Joystick _joy = new Joystick(0);

	/* a couple latched values to detect on-press events for buttons and POV */
	boolean[] _btns = new boolean[Constants.kNumButtonsPlusOne];
	boolean[] btns = new boolean[Constants.kNumButtonsPlusOne];

	void InitRobot() {

		_motorCntrller.set(ControlMode.PercentOutput, 0);

		/* pick directions */
		_motorCntrller.setSensorPhase(false);
		_motorCntrller.setInverted(false);

	}
	
	void SelectLimitSwitch(int choice) {
		if (choice == 0) {
			/* use feedback connector but disable feature, use-webdash to reenable */
			_motorCntrller.configForwardLimitSwitchSource(	LimitSwitchSource.FeedbackConnector,
													LimitSwitchNormal.Disabled,
													Constants.kTimeoutMs);

			_motorCntrller.configReverseLimitSwitchSource(	LimitSwitchSource.FeedbackConnector,
													LimitSwitchNormal.Disabled,
													Constants.kTimeoutMs);

			System.out.println("Limit Switches disabled.");
		} else if (choice == 1) {
			/* use feedback connector - use three functions */
			_motorCntrller.configForwardLimitSwitchSource(	LimitSwitchSource.FeedbackConnector,
													LimitSwitchNormal.NormallyOpen,
													Constants.kTimeoutMs);

			_motorCntrller.configReverseLimitSwitchSource(	LimitSwitchSource.FeedbackConnector,
													LimitSwitchNormal.NormallyOpen,
													Constants.kTimeoutMs);

			System.out.println("Limit Switches locally enabled.");
		} else if (choice == 2) {
			/* use remote CANifier - use four param functions */
			_motorCntrller.configForwardLimitSwitchSource(	RemoteLimitSwitchSource.RemoteCANifier,
													LimitSwitchNormal.NormallyOpen,
													_canifLimits.getDeviceID(),
													Constants.kTimeoutMs);

			_motorCntrller.configReverseLimitSwitchSource(	RemoteLimitSwitchSource.RemoteCANifier,
													LimitSwitchNormal.NormallyOpen,
													_canifLimits.getDeviceID(),
													Constants.kTimeoutMs);

			System.out.println("Remote Limit Switches enabled using CANifier.");
		}  else if (choice == 3) {
			/* use remote Talon - use four param functions */
			_motorCntrller.configForwardLimitSwitchSource(	RemoteLimitSwitchSource.RemoteTalonSRX,
													LimitSwitchNormal.NormallyOpen,
													_talonLimits.getDeviceID(),
													Constants.kTimeoutMs);

			_motorCntrller.configReverseLimitSwitchSource(	RemoteLimitSwitchSource.RemoteTalonSRX,
													LimitSwitchNormal.NormallyOpen,
													_talonLimits.getDeviceID(),
													Constants.kTimeoutMs);

			System.out.println("Remote Limit Switches enabled using another Talon SRX.");
		}
	}
	
	
	void CommonLoop() {

		/* grab the joystick inputs */
		getButtons(btns);

		double joyForward = -1 * _joy.getY(); /* positive stick => forward */

		/* deadband the sticks */
		joyForward = deadband(joyForward);

		/* button 1*/
		if (btns[1] && !_btns[1]) {
			/* if button1 is just pressed */
			SelectLimitSwitch(1);
		}
		if (btns[2] && !_btns[2]) {
			/* if button2 is just pressed */
			SelectLimitSwitch(2);
		}
		if (btns[3] && !_btns[3]) {
			/* if button3 is just pressed */
			SelectLimitSwitch(3);
		}
		if (btns[4] && !_btns[4]) {
			/* if button4 is just pressed */
			SelectLimitSwitch(4);
		}
		if (btns[6] && !_btns[6]) {
			/* top right shoulder button - don't neutral motor if remote limit source is not available */
			int value = 1;
			_motorCntrller.configSetParameter(ParamEnum.eLimitSwitchDisableNeutralOnLOS, value, 0x00, 0x00, Constants.kTimeoutMs);

			System.out.println("Checking disabled for sensor presence");
		}
		if (btns[8] && !_btns[8]) {
			/* btm right shoulder button - neutral motor if remote limit source is not available */
			int value = 0;
			_motorCntrller.configSetParameter(ParamEnum.eLimitSwitchDisableNeutralOnLOS, value, 0x00, 0x00, Constants.kTimeoutMs);

			System.out.println("Checking enabled for sensor presence");
		}
		copyButtons(_btns, btns);

		/* drive talon with gamepad */
		_motorCntrller.set(ControlMode.PercentOutput, joyForward);
	}
	
	//------------------------- Loops -------------------------------//
		public void disabledInit() {
			/* initialize hardware so that sensor phases on set before Teleop.
			 * This makes self-test more useful. */
			InitRobot();
		}
		public void disabledPeriodic() {
			CommonLoop();
		}
		public void teleopInit() {
			/* initialize hardware at start of teleop, just in case Talon was replaced / field-upgraded during disable.
			 * All params are persistent except for status frame periods. */
			InitRobot();
		}
		public void teleopPeriodic() {
			CommonLoop();
		}

		//-------------- Some helpful routines ---------------//
		void getButtons(boolean[] btns) {
			for (int i = 1; i < Constants.kNumButtonsPlusOne; ++i) {
				btns[i] = _joy.getRawButton(i);
			}
		}
		void copyButtons(boolean[] destination, boolean[] source) {
			for (int i = 1; i < Constants.kNumButtonsPlusOne; ++i) {
				destination[i] = source[i];
			}
		}
		double deadband(double value) {
			if (value >= +0.05) {
				return value;
			}
			if (value <= -0.05) {
				return value;
			}
			return 0;
		}
}
