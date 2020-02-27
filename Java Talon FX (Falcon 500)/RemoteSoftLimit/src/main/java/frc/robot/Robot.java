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

/**
 * Description:
 * The RemoteSoftLimit example demonstrates the new Talon and Victor Remote Features, which
 * allow for the use of remote Software Limit Switches. The project allows for quick change in
 * sensor configuration between:
 * 1.) Local Integrated Sensor				[Local Feedback]
 * 2.) Remote Talon Sensor       			[Talon]     
 * 3.) Remote CANifier Quadrature Encoder 	[CANifier] 
 * 4.) Gadgeteer Pigeon Yaw Reading 		[Remote SRX Feedback]       
 * 5.) Remote Pigeon Yaw Reading 			[CAN Pigeon]        
 * 6.) CANifier PWM input 1 				[CANifier]
 * The project also allows for enable/disable on soft limit triggering when sensor is not 
 * present from remote sources. configSoftLimitDisableNeutralOnLOS only works for remote sensors.
 * 
 * This project is a quick overview on possible different soft limit sources and the
 * ability to enable/disable neutral output when remote limit source can't be found. 
 * 
 * Controls:
 * Current Sensor Configurtion is indicated by print message in terminal
 * 1.) Local Integrated Sensor				[Local Feedback]		Button 1
 * 2.) Remote Talon Sensor       			[Talon] 				Button 2
 * 3.) Remote CANifier Quadrature Encoder 	[CANifier] 				Button 3
 * 4.) Gadgeteer Pigeon Yaw Reading 		[Remote SRX Feedback]  	Button 4
 * 5.) Remote Pigeon Yaw Reading 			[CAN Pigeon]        	Button 5
 * 6.) CANifier PWM input 1 				[CANifier]				Button 7
 *
 * 7.) Disable Neutral Output on loss of remote limit source        Button 6
 * 8.) Enable Neutral Output on loss of remote limit source         Button 8
 * 
 * Left Joystick Y-Axis: Throttle Talon/Motor forward and reverse in Percent Output
 *
 * Supported Version:
 * 	- Talon FX: 20.2.3.0
 *  - CANifier: 20.0
 *  - Pigeon IMU: 20.0
 */
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifierStatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;

public class Robot extends TimedRobot {
    /* Hardware */
    TalonFX _motorCntrller = new TalonFX(1);	// Victor SPX can be used with remote sensor features.
    CANifier _canifLimits = new CANifier(0);	// Use this CANifier for remote limit switches
    TalonFX _talonLimits = new TalonFX(2); 	// Use this Talon for remote limit switches
    PigeonIMU _imu = new PigeonIMU(3);
    Joystick _joy = new Joystick(0);

    /* a couple latched values to detect on-press events for buttons and POV */
    boolean[] _currentBtns = new boolean[Constants.kNumButtonsPlusOne];
    boolean[] _previousBtns = new boolean[Constants.kNumButtonsPlusOne];

    void initRobot() {
        /* Set robot output to neutral at start */
        _motorCntrller.set(TalonFXControlMode.PercentOutput, 0);

        /* Factory Default all hardware to prevent unexpected behaviour */
		_motorCntrller.configFactoryDefault();
		_canifLimits.configFactoryDefault();
		_talonLimits.configFactoryDefault();
		_imu.configFactoryDefault();

        /* pick directions */
        _motorCntrller.setSensorPhase(true);
        _motorCntrller.setInverted(false);
		/*
		 * Talon FX does not need sensor phase set for its integrated sensor
		 * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
		 * and the user calls getSelectedSensor* to get the sensor's position/velocity.
		 * 
		 * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
		 */
        // _motorCntrller.setSensorPhase(true);
        // _talonLimits.setSensorPhase(true);

        /* use feedback connector but disable feature, use-webdash to reenable */
        _motorCntrller.configForwardSoftLimitEnable(true, Constants.kTimeoutMs);
        _motorCntrller.configReverseSoftLimitEnable(true, Constants.kTimeoutMs);

        /* speed up CANifier frames related to signals sunk by Talon/Victor */
        /* speed up quadrature pos/vel */
        _canifLimits.setStatusFramePeriod(CANifierStatusFrame.Status_2_General, 10, Constants.kTimeoutMs);
        /* speed up PWM1 */
        _canifLimits.setStatusFramePeriod(CANifierStatusFrame.Status_4_PwmInputs1, 10, Constants.kTimeoutMs);

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
            _motorCntrller.configRemoteFeedbackFilter(  0x00, /* device ID does not matter since filter is off */
                                                        RemoteSensorSource.Off, 
                                                        Constants.REMOTE_0, 
                                                        Constants.kTimeoutMs);

            /* not using remote 1 - turn it off to prevent remote LossOfSignal (LOS) fault. */
            _motorCntrller.configRemoteFeedbackFilter(  0x00, /* device ID does not matter since filter is off */
                                                        RemoteSensorSource.Off, 
                                                        Constants.REMOTE_1, 
                                                        Constants.kTimeoutMs);

            /* select local quadrature if using Talon FX */
            _motorCntrller.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 
                                                        Constants.PID_PRIMARY,
                                                        Constants.kTimeoutMs);
                                                        
            /* select limits */
            _motorCntrller.configForwardSoftLimitThreshold(Constants.kForwardSoftLimit_Quad, Constants.kTimeoutMs);
            _motorCntrller.configReverseSoftLimitThreshold(Constants.kReverseSoftLimit_Quad, Constants.kTimeoutMs);

            System.out.println("Using local integrated sensor.");
        } else if (choice == 2) {
            /* select a quadrature encoder connected to a remote Talon */
            _motorCntrller.configRemoteFeedbackFilter(  _talonLimits.getDeviceID(),
                                                        RemoteSensorSource.TalonFX_SelectedSensor, 
                                                        Constants.REMOTE_0, 
                                                        Constants.kTimeoutMs);

            /* not using remote 1 */
            _motorCntrller.configRemoteFeedbackFilter(  0x00, /* device ID does not matter since filter is off */
                                                        RemoteSensorSource.Off, 
                                                        Constants.REMOTE_1, 
                                                        Constants.kTimeoutMs);

            _motorCntrller.configSelectedFeedbackSensor(TalonFXFeedbackDevice.RemoteSensor0, 
                                                        Constants.PID_PRIMARY,
                                                        Constants.kTimeoutMs);

            /* select limits */
            _motorCntrller.configForwardSoftLimitThreshold(Constants.kForwardSoftLimit_Quad, Constants.kTimeoutMs);
            _motorCntrller.configReverseSoftLimitThreshold(Constants.kReverseSoftLimit_Quad, Constants.kTimeoutMs);

            System.out.println("Using remote Talon's sensor.");
        } else if (choice == 3) {
            /* select a quadrature encoder connected to a CANifier */
            _motorCntrller.configRemoteFeedbackFilter(  _canifLimits.getDeviceID(),
                                                        RemoteSensorSource.CANifier_Quadrature, 
                                                        Constants.REMOTE_0, 
                                                        Constants.kTimeoutMs);

            /* not using remote 1 */
            _motorCntrller.configRemoteFeedbackFilter(  0x00, /* device ID does not matter since filter is off */
                                                        RemoteSensorSource.Off,
                                                        Constants.REMOTE_1, 
                                                        Constants.kTimeoutMs);

            /* select remote 0 for sensor features */
            _motorCntrller.configSelectedFeedbackSensor(TalonFXFeedbackDevice.RemoteSensor0, 
                                                        Constants.PID_PRIMARY,
                                                        Constants.kTimeoutMs);

            /* select limits */
            _motorCntrller.configForwardSoftLimitThreshold(Constants.kForwardSoftLimit_Quad, Constants.kTimeoutMs);
            _motorCntrller.configReverseSoftLimitThreshold(Constants.kReverseSoftLimit_Quad, Constants.kTimeoutMs);

            System.out.println("Using remote CANifier's quadrature encoder.");
        } else if (choice == 4) {
            /* select a ribbon-cabled Pigeon that is ribbon cabled to a remote Talon. */
            _motorCntrller.configRemoteFeedbackFilter(  _talonLimits.getDeviceID(),
                                                        RemoteSensorSource.GadgeteerPigeon_Yaw, 
                                                        Constants.REMOTE_0, 
                                                        Constants.kTimeoutMs);
            /* not using remote 1 */
            _motorCntrller.configRemoteFeedbackFilter(  0x00, /* device ID does not matter since filter is off */
                                                        RemoteSensorSource.Off, 
                                                        Constants.REMOTE_1, 
                                                        Constants.kTimeoutMs);

            _motorCntrller.configSelectedFeedbackSensor(TalonFXFeedbackDevice.RemoteSensor0, 
                                                        Constants.PID_PRIMARY,
                                                        Constants.kTimeoutMs);

            /* select limits */
            _motorCntrller.configForwardSoftLimitThreshold(Constants.kForwardSoftLimit_Pigeon, Constants.kTimeoutMs);
            _motorCntrller.configReverseSoftLimitThreshold(Constants.kReverseSoftLimit_Pigeon, Constants.kTimeoutMs);

            System.out.println("Using remote Pigeon Yaw that is plugged into a remote Talon.");
        } else if (choice == 5) {
            /* turn off remote 0 */
            _motorCntrller.configRemoteFeedbackFilter(  0x00, /* device ID does not matter since filter is off */
                                                        RemoteSensorSource.Off, 
                                                        Constants.REMOTE_0, 
                                                        Constants.kTimeoutMs);

            /* select a Pigeon on CAN Bus. */
            _motorCntrller.configRemoteFeedbackFilter(  _imu.getDeviceID(), 
                                                        RemoteSensorSource.Pigeon_Yaw,
                                                        Constants.REMOTE_1, /* use remote filter 1 this time */
                                                        Constants.kTimeoutMs);

            _motorCntrller.configSelectedFeedbackSensor(TalonFXFeedbackDevice.RemoteSensor1, /* Use remote filter 1 this time */
                                                        Constants.PID_PRIMARY,
                                                        Constants.kTimeoutMs);

            /* select limits */
            _motorCntrller.configForwardSoftLimitThreshold(Constants.kForwardSoftLimit_Pigeon, Constants.kTimeoutMs);
            _motorCntrller.configReverseSoftLimitThreshold(Constants.kReverseSoftLimit_Pigeon, Constants.kTimeoutMs);

            System.out.println("Using remote Pigeon that is CAN bus connected.");
        } else if (choice == 7) {

            /* turn off remote 0 */
            _motorCntrller.configRemoteFeedbackFilter(  0x00, /* device ID does not matter since filter is off */
                                                        RemoteSensorSource.Off, 
                                                        Constants.REMOTE_0, 
                                                        Constants.kTimeoutMs);

            /* select a Pigeon on CAN Bus. */
            _motorCntrller.configRemoteFeedbackFilter(  _canifLimits.getDeviceID(), 
                                                        RemoteSensorSource.CANifier_PWMInput0,
                                                        Constants.REMOTE_1, /* use remote filter 1 this time */
                                                        Constants.kTimeoutMs);

            _motorCntrller.configSelectedFeedbackSensor(TalonFXFeedbackDevice.RemoteSensor1, /* use remote filter 1 this time */
                                                        Constants.PID_PRIMARY, 
                                                        Constants.kTimeoutMs);

            /* select limits */
            _motorCntrller.configForwardSoftLimitThreshold(Constants.kForwardSoftLimit_PWMInput, Constants.kTimeoutMs);
            _motorCntrller.configReverseSoftLimitThreshold(Constants.kReverseSoftLimit_PWMInput, Constants.kTimeoutMs);

            System.out.println("Using remote CANifier PWM input 1.");
        }
    }

    void commonLoop() {
        /* Gamepad processing */
        getButtons(_currentBtns);               // Update buttons
        double joyForward = -1 * _joy.getY();   // positive stick => forward
        joyForward = deadband(joyForward);      // Deadband joystick
        joyForward *= 0.50f;                    // Reduce speed so PWM sensor doesn't wrap around

        /* Select Soft Limit Setup based on button pressed */
        if (_currentBtns[1] && !_previousBtns[1]) {selectSoftLimitSetup(1);} // Button 1
        if (_currentBtns[2] && !_previousBtns[2]) {selectSoftLimitSetup(2);} // Button 2
        if (_currentBtns[3] && !_previousBtns[3]) {selectSoftLimitSetup(3);} // Button 3
        if (_currentBtns[4] && !_previousBtns[4]) {selectSoftLimitSetup(4);} // Button 4
        if (_currentBtns[5] && !_previousBtns[5]) {selectSoftLimitSetup(5);} // Button 5
        if (_currentBtns[7] && !_previousBtns[7]) {selectSoftLimitSetup(7);} // Button 7

        /* Enable /Disable Software Limit Switch Triggering on Loss of Sensor Presense */
        if (_currentBtns[6] && !_previousBtns[6]) {                          // Button 6
			/* Don't neutral motor if remote limit source is not available */
            _motorCntrller.configSoftLimitDisableNeutralOnLOS(true, Constants.kTimeoutMs);

            System.out.println("Checking disabled for sensor presence.");
        }
        if (_currentBtns[8] && !_previousBtns[8]) {                          // Button 8
			/* Neutral motor if remote limit source is not available */
            _motorCntrller.configSoftLimitDisableNeutralOnLOS(false, Constants.kTimeoutMs);

            System.out.println("Checking enabled for sensor presence.");
        }
		/* Copy current buttons into previous buttons for single button press tracking */
        System.arraycopy(_currentBtns, 0, _previousBtns, 0, _currentBtns.length);

        /* Percent Output drive Talon with gamepad */
        _motorCntrller.set(TalonFXControlMode.PercentOutput, joyForward);
    }

    // ------------------------- Loops -------------------------------//
    @Override
    public void disabledInit() {
        /**
         * Initialize hardware so that sensor phases on set before Teleop. This makes
         * self-test more useful.
         */
        initRobot();
    }

    @Override
    public void disabledPeriodic() {
        commonLoop();
    }

    @Override
    public void teleopInit() {
        /**
         * Initialize hardware at start of teleop, just in case Talon was replaced or
         * field-upgraded during disable. All params are persistent except for status
         * frame periods.
         */
        initRobot();
    }

    @Override
    public void teleopPeriodic() {
        commonLoop();
    }

    // -------------- Some helpful routines ---------------//
    void getButtons(boolean[] _currentBtns) {
        for (int i = 1; i < Constants.kNumButtonsPlusOne; ++i) {
            _currentBtns[i] = _joy.getRawButton(i);
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
