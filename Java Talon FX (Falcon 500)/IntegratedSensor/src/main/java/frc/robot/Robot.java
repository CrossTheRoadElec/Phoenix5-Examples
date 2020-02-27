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
/*
 * Description: 	Example on how to poll the integrated sensor from the Talon FX (Falcon 500).
 */
package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {
	/*
	 * Talon FX has 2048 units per revolution
	 * 
	 * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-resolution
	 */
	final int kUnitsPerRevolution = 2048; /* this is constant for Talon FX */

	/**
	 * Decide if positive motor-output/sensor-velocity should be when motor spins
	 * clockwise or counter-clockwise.
	 */
	final TalonFXInvertType kInvertType = TalonFXInvertType.CounterClockwise; // <<< What direction you want "forward/up" to be.

	/** electic brake during neutral */
	final NeutralMode kBrakeDurNeutral = NeutralMode.Coast;

	/** Talon to control and monitor */
	TalonFX _talon = new TalonFX(1); // <<< Choose the Talon ID - check in Tuner to see what ID it is.

	/** user joystick for basic control */
	Joystick _joy = new Joystick(0);

	/** print every few loops */
	int _loops = 0;

	/**
	 * This function is called once on roboRIO bootup Select the quadrature/mag
	 * encoder relative sensor
	 */
	public void robotInit() {

		/*
		 * two styles of configs, older API that requires a config all per setting, or
		 * newer API that has everything in one call.
		 */
		if (false) {
			/* older legacy config API example */
			_talon.configFactoryDefault();
			/* select integ-sensor for PID0 (it doesn't matter if PID is actually used) */
			_talon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
		} else {
			/* newer config API */
			TalonFXConfiguration configs = new TalonFXConfiguration();
			/* select integ-sensor for PID0 (it doesn't matter if PID is actually used) */
			configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
			/* config all the settings */
			_talon.configAllSettings(configs);
		}

		/*
		 * status frame rate - user can speed up the position/velocity reporting if need
		 * be.
		 * https://phoenix-documentation.readthedocs.io/en/latest/ch18_CommonAPI.html#
		 * status-groups Keep an eye on the CAN bus utilization in the DriverStation if
		 * this is used.
		 * https://phoenix-documentation.readthedocs.io/en/latest/ch18_CommonAPI.html#
		 * can-bus-utilization-error-metrics
		 */
		_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);

		/*
		 * choose which direction motor should spin during positive
		 * motor-output/sensor-velocity. Note setInverted also takes classic true/false
		 * as an input.
		 */
		_talon.setInverted(kInvertType);
		/*
		 * Talon FX does not need sensor phase set for its integrated sensor
		 * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
		 * and the user calls getSelectedSensor* to get the sensor's position/velocity.
		 * 
		 * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
		 */
		//_talon.setSensorPhase(true);

		/* brake or coast during neutral */
		_talon.setNeutralMode(kBrakeDurNeutral);
	}

	/**
	 * Get the selected sensor register and print it
	 */
	public void robotPeriodic() {

		/* drive talon based on joystick */
		_talon.set(ControlMode.PercentOutput, _joy.getY());

		/* get the selected sensor for PID0 */
		double appliedMotorOutput = _talon.getMotorOutputPercent();
		int selSenPos = _talon.getSelectedSensorPosition(0); /* position units */
		int selSenVel = _talon.getSelectedSensorVelocity(0); /* position units per 100ms */

		/* scaling depending on what user wants */
		double pos_Rotations = (double) selSenPos / kUnitsPerRevolution;
		double vel_RotPerSec = (double) selSenVel / kUnitsPerRevolution * 10; /* scale per100ms to perSecond */
		double vel_RotPerMin = vel_RotPerSec * 60.0;

		/*
		 * Print to console. This is also a good oppurtunity to self-test/plot in Tuner
		 * to see how the values match.
		 * 
		 * Note these prints can cause "Loop time of 0.02s overrun" errors in the console.
		 * This is because prints are slow.
		 */
		if (++_loops >= 10) {
			_loops = 0;
			System.out.printf("Motor-out: %.2f | ", appliedMotorOutput);
			System.out.printf("Pos-units: %d | ", selSenPos);
			System.out.printf("Vel-unitsPer100ms: %d | ", selSenVel);
			System.out.printf("Pos-Rotations:%.3f | ", pos_Rotations);
			System.out.printf("Vel-RPS:%.1f | ", vel_RotPerSec);
			System.out.printf("Vel-RPM:%.1f | ", vel_RotPerMin);
			System.out.println();
		}

		/* set position to zero on button 1 */
		if (_joy.getRawButton(1)) {
			_talon.setSelectedSensorPosition(0);
		}
	}
}
