/**
 * Example demonstrating the motion magic control mode.
 * Tested with Logitech F710 USB Gamepad inserted into Driver Station.
 * 
 * Be sure to select the correct feedback sensor using SetFeedbackDevice() below.
 *
 * After deploying/debugging this to your RIO, first use the left Y-stick 
 * to throttle the Talon manually.  This will confirm your hardware setup/sensors
 * and will allow you to take initial measurements.
 * 
 * Be sure to confirm that when the Talon is driving forward (green) the 
 * position sensor is moving in a positive direction.  If this is not the 
 * cause, flip the boolean input to the reverseSensor() call below.
 *
 * Once you've ensured your feedback device is in-phase with the motor,
 * and followed the walk-through in the Talon SRX Software Reference Manual,
 * use button1 to motion-magic servo to target position specified by the gamepad stick.
 */
package org.usfirst.frc.team217.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Joystick.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.*;

public class Robot extends IterativeRobot {
	TalonSRX _talon = new TalonSRX(3);
	Joystick _joy = new Joystick(0);
	StringBuilder _sb = new StringBuilder();

	public void robotInit() {
		/* first choose the sensor */
		_talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		_talon.setInverted(true);
		// _talon.configEncoderCodesPerRev(XXX), // if using
		// FeedbackDevice.QuadEncoder
		// _talon.configPotentiometerTurns(XXX), // if using
		// FeedbackDevice.AnalogEncoder or AnalogPot

		/* set the peak and nominal outputs, 12V means full */
		_talon.configNominalOutputForward(0, 10);
		_talon.configNominalOutputReverse(0, 10);
		_talon.configPeakOutputForward(1, 10);
		
		/* set closed loop gains in slot0 - see documentation */
		_talon.selectProfileSlot(0, 10);
		_talon.config_kF(0, 0, 10);
		_talon.config_kP(0, 0, 10);
		_talon.config_kI(0, 0, 10);
		_talon.config_kD(0, 0, 10);
		/* set acceleration and vcruise velocity - see documentation */
		_talon.configMotionCruiseVelocity(0, 10);
		_talon.configMotionAcceleration(0, 10);
	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
		/* get gamepad axis - forward stick is positive */
		double leftYstick = -1.0 * _joy.getAxis(AxisType.kY);
		/* calculate the percent motor output */
		double motorOutput = _talon.getMotorOutputVoltage() / _talon.getBusVoltage();
		/* prepare line to print */
		_sb.append("\tout:");
		_sb.append(motorOutput);
		_sb.append("\tspd:");
		_sb.append(_talon.getSelectedSensorVelocity(0));

		if (_joy.getRawButton(1)) {
			/* Motion Magic */
			double targetPos = leftYstick
					* 10.0; /* 10 Rotations in either direction */
			_talon.set(ControlMode.MotionMagic, targetPos); 

			/* append more signals to print when in speed mode. */
			_sb.append("\terr:");
			_sb.append(_talon.getClosedLoopError(0));
			_sb.append("\ttrg:");
			_sb.append(targetPos);
		} else {
			/* Percent voltage mode */
			_talon.set(ControlMode.PercentOutput, leftYstick);
		}
		/* instrumentation */
		Instrum.Process(_talon, _sb);
	}
}
