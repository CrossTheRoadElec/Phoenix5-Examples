/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4130.robot;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifierStatusFrame;
import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

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
	TalonSRX  _motorCntrller = new TalonSRX(2); // could also be Victor SPX if using remote sensor features.

	CANifier  _canifLimits = new CANifier(2); /* use this CANifier for limit switches */
	TalonSRX  _talonLimits = new TalonSRX(5); /* use this Talon for limit switches */

	Joystick _joy = new Joystick(0);

	PigeonIMU  _imu = new PigeonIMU(3);

	/* a couple latched values to detect on-press events for buttons and POV */
	boolean[] _btns = new boolean[Constants.kNumButtonsPlusOne];
	boolean[] btns = new boolean[Constants.kNumButtonsPlusOne];
	
	void initRobot() {

		_motorCntrller.set(ControlMode.PercentOutput, 0);

		/* pick directions */
		_motorCntrller.setSensorPhase(true);
		_motorCntrller.setInverted(false);

		/* use feedback connector but disable feature, use-webdash to reenable */
		_motorCntrller.configForwardSoftLimitEnable(true, Constants.kTimeoutMs);
		_motorCntrller.configReverseSoftLimitEnable(true, Constants.kTimeoutMs);

		/* speed up CANifier frames related to signals sunk by Talon/Victor */
		_canifLimits.setStatusFramePeriod(CANifierStatusFrame.Status_2_General, 10, Constants.kTimeoutMs); /* speed up quadrature pos/vel */
		_canifLimits.setStatusFramePeriod(CANifierStatusFrame.Status_4_PwmInputs1, 10, Constants.kTimeoutMs); /* speed up PWM1 */

		/* Pick local quadrature to start with */
		selectSoftLimitSetup(1);
	}
	
	/**
	 * General setup requires
	 * - configure remote filter 0 (if used)
	 * - configure remote filter 1 (if used)
	 * - select remote 0 or remote 1 sensor
	 * - pick soft limit thresholds
	 * - enable soft limit (done in InitRobot).
	 */
	void selectSoftLimitSetup(int choice) {
		if (choice == 1) {

			/* not using remote 0 - turn it off to prevent remote LossOfSignal (LOS) fault. */
			_motorCntrller.configRemoteFeedbackFilter(	0x00, /* device ID does not matter since filter is off */
														RemoteSensorSource.Off,
														Constants.REMOTE_0,
														Constants.kTimeoutMs);

			/* not using remote 1 - turn it off to prevent remote LossOfSignal (LOS) fault. */
			_motorCntrller.configRemoteFeedbackFilter(	0x00, /* device ID does not matter since filter is off */
														RemoteSensorSource.Off,
														Constants.REMOTE_1,
														Constants.kTimeoutMs);

			/* select local quadrature if using Talon SRX */
			_motorCntrller.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
														Constants.PID_PRIMARY,
														Constants.kTimeoutMs);

			/* select limits */
			_motorCntrller.configForwardSoftLimitThreshold(Constants.kForwardSoftLimit_Quad, Constants.kTimeoutMs);
			_motorCntrller.configReverseSoftLimitThreshold(Constants.kReverseSoftLimit_Quad, Constants.kTimeoutMs);

			System.out.println("Using local quadrature encoder.");
		} else if (choice == 2) {
			/* select a quadrature encoder connected to a remote Talon */
			_motorCntrller.configRemoteFeedbackFilter(	_talonLimits.getDeviceID(),
														RemoteSensorSource.TalonSRX_SelectedSensor,
														Constants.REMOTE_0,
														Constants.kTimeoutMs);

			/* not using remote 1 */
			_motorCntrller.configRemoteFeedbackFilter(	0x00, /* device ID does not matter since filter is off */
														RemoteSensorSource.Off,
														Constants.REMOTE_1,
														Constants.kTimeoutMs);

			_motorCntrller.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0,
														Constants.PID_PRIMARY,
														Constants.kTimeoutMs);

			/* select limits */
			_motorCntrller.configForwardSoftLimitThreshold(Constants.kForwardSoftLimit_Quad, Constants.kTimeoutMs);
			_motorCntrller.configReverseSoftLimitThreshold(Constants.kReverseSoftLimit_Quad, Constants.kTimeoutMs);

			System.out.println("Using remote Talon's quadrature encoder.");
		} else if (choice == 3) {
			/* select a quadrature encoder connected to a CANifier */
			_motorCntrller.configRemoteFeedbackFilter(	_canifLimits.getDeviceID(),
														RemoteSensorSource.CANifier_Quadrature,
														Constants.REMOTE_0,
														Constants.kTimeoutMs);

			/* not using remote 1 */
			_motorCntrller.configRemoteFeedbackFilter(	0x00, /* device ID does not matter since filter is off */
														RemoteSensorSource.Off,
														Constants.REMOTE_1,
														Constants.kTimeoutMs);

			/* select remote 0 for sensor features */
			_motorCntrller.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0,
														Constants.PID_PRIMARY,
														Constants.kTimeoutMs);

			/* select limits */
			_motorCntrller.configForwardSoftLimitThreshold(Constants.kForwardSoftLimit_Quad, Constants.kTimeoutMs);
			_motorCntrller.configReverseSoftLimitThreshold(Constants.kReverseSoftLimit_Quad, Constants.kTimeoutMs);

			System.out.println("Using remote CANifier's quadrature encoder.");
		} else if (choice == 4) {
			/* select a ribbon-cabled Pigeon that is ribbon cabled to a remote Talon. */
			_motorCntrller.configRemoteFeedbackFilter(	_talonLimits.getDeviceID(),
														RemoteSensorSource.GadgeteerPigeon_Yaw,
														Constants.REMOTE_0,
														Constants.kTimeoutMs);
			/* not using remote 1 */
			_motorCntrller.configRemoteFeedbackFilter(	0x00, /* device ID does not matter since filter is off */
														RemoteSensorSource.Off,
														Constants.REMOTE_1,
														Constants.kTimeoutMs);

			_motorCntrller.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0,
														Constants.PID_PRIMARY,
														Constants.kTimeoutMs);

			/* select limits */
			_motorCntrller.configForwardSoftLimitThreshold(Constants.kForwardSoftLimit_Pigeon, Constants.kTimeoutMs);
			_motorCntrller.configReverseSoftLimitThreshold(Constants.kReverseSoftLimit_Pigeon, Constants.kTimeoutMs);

			System.out.println("Using remote Pigeon Yaw that is plugged into a remote Talon.");
		} else if (choice == 5) {

			/* turn off remote 0 */
			_motorCntrller.configRemoteFeedbackFilter(	0x00, /* device ID does not matter since filter is off */
														RemoteSensorSource.Off,
														Constants.REMOTE_0,
														Constants.kTimeoutMs);

			/* select a Pigeon on CAN Bus. */
			_motorCntrller.configRemoteFeedbackFilter(	_imu.getDeviceID(),
														RemoteSensorSource.Pigeon_Yaw,
														Constants.REMOTE_1, /* use remote filter 1 this time */
														Constants.kTimeoutMs);

			_motorCntrller.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor1, /* use remote filter 1 this time */
														Constants.PID_PRIMARY, Constants.kTimeoutMs);

			/* select limits */
			_motorCntrller.configForwardSoftLimitThreshold(Constants.kForwardSoftLimit_Pigeon, Constants.kTimeoutMs);
			_motorCntrller.configReverseSoftLimitThreshold(Constants.kReverseSoftLimit_Pigeon, Constants.kTimeoutMs);

			System.out.println("Using remote Pigeon that is CAN bus connected.");
		} else if (choice == 7) {

			/* turn off remote 0 */
			_motorCntrller.configRemoteFeedbackFilter(	0x00, /* device ID does not matter since filter is off */
														RemoteSensorSource.Off,
														Constants.REMOTE_0,
														Constants.kTimeoutMs);

			/* select a Pigeon on CAN Bus. */
			_motorCntrller.configRemoteFeedbackFilter(	_canifLimits.getDeviceID(),
														RemoteSensorSource.CANifier_PWMInput1,
														Constants.REMOTE_1, /* use remote filter 1 this time */
														Constants.kTimeoutMs);

			_motorCntrller.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor1, /* use remote filter 1 this time */
														Constants.PID_PRIMARY, Constants.kTimeoutMs);

			/* select limits */
			_motorCntrller.configForwardSoftLimitThreshold(Constants.kForwardSoftLimit_PWMInput, Constants.kTimeoutMs);
			_motorCntrller.configReverseSoftLimitThreshold(Constants.kReverseSoftLimit_PWMInput, Constants.kTimeoutMs);

			System.out.println("Using remote CANifier PWM input 1.");
		}
	}

	void commonLoop() {

		/* grab the joystick inputs */
		getButtons(btns);

		double joyForward = -1 * _joy.getY(); /* positive stick => forward */

		/* deadband the sticks */
		joyForward = deadband(joyForward);

		/* button 1*/
		if (btns[1] && !_btns[1]) {
			/* if button1 is just pressed */
			selectSoftLimitSetup(1);
		}
		if (btns[2] && !_btns[2]) {
			/* if button2 is just pressed */
			selectSoftLimitSetup(2);
		}
		if (btns[3] && !_btns[3]) {
			/* if button3 is just pressed */
			selectSoftLimitSetup(3);
		}
		if (btns[4] && !_btns[4]) {
			/* if button4 is just pressed */
			selectSoftLimitSetup(4);
		}
		if (btns[5] && !_btns[5]) {
			/* if button5 is just pressed */
			selectSoftLimitSetup(5);
		}
		if (btns[7] && !_btns[7]) {
			/* if button7 is just pressed */
			selectSoftLimitSetup(7);
		}

		if (btns[6] && !_btns[6]) {
			/* top right shoulder button - don't neutral motor if remote limit source is not available */
			int value = 1;
			_motorCntrller.configSetParameter(	ParamEnum.eSoftLimitDisableNeutralOnLOS,
												value,
												0x00,
												0x00,
												Constants.kTimeoutMs);

			System.out.println("Checking disabled for sensor presence");
		}
		if (btns[8] && !_btns[8]) {
			/* btm right shoulder button - neutral motor if remote limit source is not available */
			int value = 0;
			_motorCntrller.configSetParameter(	ParamEnum.eSoftLimitDisableNeutralOnLOS,
												value,
												0x00,
												0x00,
												Constants.kTimeoutMs);

			System.out.println("Checking enabled for sensor presence");
		}

		copyButtons(_btns, btns);

		/* drive talon with gamepad */
		_motorCntrller.set(ControlMode.PercentOutput, joyForward);
	}

	//------------------------- Loops -------------------------------//
	@Override
	public void disabledInit() {
		/* initialize hardware so that sensor phases on set before Teleop.
		 * This makes self-test more useful. */
		initRobot();
	}
	@Override
	public void disabledPeriodic() {
		commonLoop();
	}
	@Override
	public void teleopInit() {
		/* initialize hardware at start of teleop, just in case Talon was replaced / field-upgraded during disable.
		 * All params are persistent except for status frame periods. */
		initRobot();
	}
	@Override
	public void teleopPeriodic() {
		commonLoop();
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
