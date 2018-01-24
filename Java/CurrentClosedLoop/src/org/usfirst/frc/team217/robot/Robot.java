package org.usfirst.frc.team217.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends IterativeRobot {

	TalonSRX _talon = new TalonSRX(3);
	Joystick _joy = new Joystick(0);
	StringBuilder _sb = new StringBuilder();
	int _loops = 0;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {

		/* set the peak and nominal outputs, 12V means full */
		_talon.configNominalOutputForward(0, Constants.kTimeoutMs);
		_talon.configNominalOutputReverse(0, Constants.kTimeoutMs);
		_talon.configPeakOutputForward(1, Constants.kTimeoutMs);
		_talon.configPeakOutputReverse(-1, Constants.kTimeoutMs);
		/*
		 * set the allowable closed-loop error, Closed-Loop output will be
		 * neutral within this range. See Table in Section 17.2.1 for native
		 * units per rotation.
		 */
		_talon.configAllowableClosedloopError(0, Constants.kPIDLoopIdx,
				Constants.kTimeoutMs); /* always servo */
		/* set closed loop gains in slot0 */
		_talon.config_kF(Constants.kPIDLoopIdx, 0.0, Constants.kTimeoutMs);
		_talon.config_kP(Constants.kPIDLoopIdx, 0.0, Constants.kTimeoutMs);
		_talon.config_kI(Constants.kPIDLoopIdx, 0.0, Constants.kTimeoutMs);
		_talon.config_kD(Constants.kPIDLoopIdx, 0.0, Constants.kTimeoutMs);

	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {

		double leftYstick = _joy.getY();
		double motorOutput = _talon.getMotorOutputPercent();
		boolean button1 = _joy.getRawButton(1);

		/* prepare line to print */
		_sb.append("\tout:");
		_sb.append(motorOutput);
		_sb.append("\tcur:");
		_sb.append(_talon.getOutputCurrent());

		if (button1) {
			_talon.set(ControlMode.Current, leftYstick * 40); // Scale to 40A
		} else {
			_talon.set(ControlMode.PercentOutput, leftYstick);
		}
		/* if Talon is in position closed-loop, print some more info */
		if (_talon.getControlMode() == ControlMode.Current) {
			/* append more signals to print when in speed mode. */
			_sb.append("\terrNative:");
			_sb.append(_talon.getClosedLoopError(0));
			_sb.append("\ttrg:");
			_sb.append(leftYstick * 40);
		}

		/*
		 * print every ten loops, printing too much too fast is generally bad
		 * for performance
		 */
		if (++_loops >= 10) {
			_loops = 0;
			System.out.println(_sb.toString());
		}
		_sb.setLength(0);
	}
}
