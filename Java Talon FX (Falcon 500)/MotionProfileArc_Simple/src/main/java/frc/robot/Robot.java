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
 * This Java FRC robot application is meant to demonstrate an example using the Motion Profile control mode
 * in Talon FX.  The TalonFX class gives us the ability to buffer up trajectory points and execute them
 * as the roboRIO streams them into the Talon FX.
 * 
 * There are many valid ways to use this feature and this example does not sufficiently demonstrate every possible
 * method.  Motion Profile streaming can be as complex as the developer needs it to be for advanced applications,
 * or it can be used in a simple fashion for fire-and-forget actions that require precise timing.
 * 
 * This application is a TimedRobot project to demonstrate a minimal implementation not requiring the command 
 * framework, however these code excerpts could be moved into a command-based project.
 * 
 * The project also includes instrumentation.java which simply has debug printfs, and a MotionProfile.java which is generated
 * in @link https://docs.google.com/spreadsheets/d/1PgT10EeQiR92LNXEOEe3VGn737P7WDP4t0CQxQgC8k0/edit#gid=1813770630&vpid=A1
 * or find Motion Profile Generator.xlsx in the Project folder.
 * 
 * Controls:
 * Button 1: When held, streams and fires the MP.  When released, contorl is back to PercentOutput Mode.
 * Button 2: Prints MP status to the console when held.
 * Left Joystick Y-Axis: Throttle Talon FX forward and reverse when not running MP.
 * Gains for Motion Profile may need to be adjusted in Constants.java
 * 
 * Steps:
 * Drive the robot normally and confirm:
 *  - Open SmartDash (or similar) to see signal values.
 *  - Talon LEDs are green when robot moves straight forward (both sides)
 *  - sen_pos_drv moves in a positive direction when robot drives straight forward.  Or use Right Talon self-test (watch PID0 sensor pos)
 *  - sen_pos_turn moves in a positive direction when robot turns left.  Or use Right Talon self-test (watch PID1 aux sensor pos)
 *  - Update constants if not using Pigeon/CTRE Mag encoders.
 *
 * Supported Versions:
 * 	- Talon FX: 20.2.3.0
 *  - Pigeon IMU: 20.0
 */
package frc.robot;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.motion.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {

    /** very simple state machine to prevent calling set() while firing MP. */
    int _state = 0;

    /** a master talon, add followers if need be. */
    TalonFX _rightMaster = new TalonFX(1);

    TalonFX _leftAuxFollower = new TalonFX(2);

    PigeonIMU _pidgy = new PigeonIMU(3);

    /** gamepad for control */
    Joystick _joy = new Joystick(0);

	/** Invert Directions for Left and Right */
	TalonFXInvertType _leftInvert = TalonFXInvertType.CounterClockwise; //Same as invert = "false"
	TalonFXInvertType _rightInvert = TalonFXInvertType.Clockwise; //Same as invert = "true"

    /** new class type in 2019 for holding MP buffer. */
    BufferedTrajectoryPointStream _bufferedStream = new BufferedTrajectoryPointStream();

    /* talon _config. */
    TalonFXConfiguration _config= new TalonFXConfiguration(); // factory default settings
    
    /* quick and dirty plotter to smartdash */
//    PlotThread _plotThread = new PlotThread(_rightMaster);

    public void robotInit() {

        /* fill our buffer object with the excel points, 
            lets do a 90 deg turn while using the profile for the robot drive*/
        initBuffer(MotionProfile.Points, MotionProfile.kNumPoints, 0.0);

        /* -------------- config the master specific settings ----------------- */
        /* remote 0 will capture Pigeon IMU */
        _config.remoteFilter1.remoteSensorDeviceID = _pidgy.getDeviceID();
        _config.remoteFilter1.remoteSensorSource = RemoteSensorSource.Pigeon_Yaw;
        /* remote 1 will capture the integrated sensor on left talon */
        _config.remoteFilter0.remoteSensorDeviceID = _leftAuxFollower.getDeviceID();
        _config.remoteFilter0.remoteSensorSource = RemoteSensorSource.TalonFX_SelectedSensor;
		/* Now that the Left sensor can be used by the master Talon,
		 * set up the Left (Aux) and Right (Master) distance into a single
		 * Robot distance as the Master's Selected Sensor 0. */
        setRobotDistanceConfigs(_rightInvert, _config);
        _config.auxiliaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.RemoteSensor1.toFeedbackDevice();
        /* rest of the configs */
        _config.neutralDeadband = Constants.kNeutralDeadband; /* 0.1 % super small for best low-speed control */
        _config.slot0.kF = Constants.kGains_MotProf.kF;
        _config.slot0.kP = Constants.kGains_MotProf.kP;
        _config.slot0.kI = Constants.kGains_MotProf.kI;
        _config.slot0.kD = Constants.kGains_MotProf.kD;
        _config.slot0.integralZone = (int) Constants.kGains_MotProf.kIzone;
        _config.slot0.closedLoopPeakOutput = Constants.kGains_MotProf.kPeakOutput;
        // _config.slot0.allowableClosedloopError // leave default
        // _config.slot0.maxIntegralAccumulator; // leave default
        // _config.slot0.closedLoopPeriod; // leave default
        _config.slot1.kF = Constants.kGains_MotProf.kF;
        _config.slot1.kP = Constants.kGains_MotProf.kP;
        _config.slot1.kI = Constants.kGains_MotProf.kI;
        _config.slot1.kD = Constants.kGains_MotProf.kD;
        _config.slot1.integralZone = (int) Constants.kGains_MotProf.kIzone;
        _config.slot1.closedLoopPeakOutput = Constants.kGains_MotProf.kPeakOutput;
        // _config.slot1.allowableClosedloopError // leave default
        // _config.slot1.maxIntegralAccumulator; // leave default
        // _config.slot1.closedLoopPeriod; // leave default
        _rightMaster.configAllSettings(_config);

        /* -------------- _config the left ----------------- */
        _leftAuxFollower.configFactoryDefault(); /* no special configs */
        _pidgy.configFactoryDefault();

        /* pick the sensor phase and desired direction */
        _leftAuxFollower.setInverted(_leftInvert);
        _rightMaster.setInverted(_rightInvert); /* right side has to apply +V to M-, to go forward */
		/*
		 * Talon FX does not need sensor phase set for its integrated sensor
		 * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
		 * and the user calls getSelectedSensor* to get the sensor's position/velocity.
		 * 
		 * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
		 */
        // _leftAuxFollower.setSensorPhase(true);
        // _rightMaster.setSensorPhase(true);

        /* speed up the target polling for PID[0] and PID-aux[1] */
        _rightMaster.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20); /* plotthread is polling aux-pid-sensor-pos */
        _rightMaster.setStatusFramePeriod(StatusFrame.Status_10_Targets, 20);
        _rightMaster.setStatusFramePeriod(StatusFrame.Status_17_Targets1, 20);
    }

    public void robotPeriodic() {
        /* get joystick button and stick */
        boolean bPrintValues = _joy.getRawButton(2);
        boolean bFireMp = _joy.getRawButton(1);
        double axis = -1.0 * _joy.getRawAxis(1); /* forward stick should be positive */
        double turn = -1.0 * _joy.getRawAxis(2); /* turn stick should be positive for turning left */

        /* if button is up, just drive the motor in PercentOutput */
        if (bFireMp == false) {
            _state = 0;
        }

        switch (_state) {
	        /* drive master talon normally */
	        case 0:
	            /* some clever use of the arbitratry feedforward to add the turn, there are many
	             * alternative ways to do this */
	            _rightMaster.set(TalonFXControlMode.PercentOutput, axis, DemandType.ArbitraryFeedForward, +turn);
	            _leftAuxFollower.set(TalonFXControlMode.PercentOutput, axis, DemandType.ArbitraryFeedForward, -turn);
	            if (bFireMp == true) {
	                /* go to MP logic */
	                _state = 1;
	            }
	            break;
	
	        /* fire the MP, and stop calling set() since that will cancel the MP */
	        case 1:
	            ZeroAllSensors();
	            _leftAuxFollower.follow(_rightMaster, FollowerType.AuxOutput1);
	            _rightMaster.startMotionProfile(_bufferedStream, 10, TalonFXControlMode.MotionProfileArc.toControlMode());
	            _state = 2;
                Instrum.printLine("MP started");
	            break;
	
	        /* wait for MP to finish */
	        case 2:
	            if (_rightMaster.isMotionProfileFinished()) {
                    Instrum.printLine("MP finished");
	                _state = 3;
	            }
	            break;
	
	        /* MP is finished, nothing to do */
	        case 3:
	            break;
        }

        /* print MP values */
        Instrum.loop(bPrintValues, _rightMaster);
    }

    /**
     * Fill _bufferedStream with points from csv/generated-table.
     *
     * @param profile  generated array from excel
     * @param totalCnt num points in profile
     */
    private void initBuffer(double[][] profile, int totalCnt, double finalTurnDeg) {

        boolean forward = true; // set to false to drive in opposite direction of profile (not really needed
                                // since you can use negative numbers in profile).

        TrajectoryPoint point = new TrajectoryPoint(); // temp for for loop, since unused params are initialized
                                                       // automatically, you can alloc just one

        /* clear the buffer, in case it was used elsewhere */
        _bufferedStream.Clear();

        /* Insert every point into buffer, no limit on size */
        for (int i = 0; i < totalCnt; ++i) {

            double direction = forward ? +1 : -1;
            /* use the generated profile to figure out the forward arc path (translation)*/
            double positionRot = profile[i][0];
            double velocityRPM = profile[i][1];
            int durationMilliseconds = (int) profile[i][2];

            /* to get the turn target, lets just scale from 0 deg to caller's final deg linearizly */
            double targetTurnDeg = finalTurnDeg * (i + 1) / totalCnt;

            /* for each point, fill our structure and pass it to API */
            point.timeDur = durationMilliseconds;

            /* drive part */
            point.position = direction * positionRot * Constants.kSensorUnitsPerRot; // Rotations => sensor units
            point.velocity = direction * velocityRPM * Constants.kSensorUnitsPerRot / 600.0; // RPM => units per 100ms
            point.arbFeedFwd = 0; // good place for kS, kV, kA, etc...

            /* turn part */
            point.auxiliaryPos = targetTurnDeg * Constants.kTurnUnitsPerDeg; // Convert deg to remote sensor units
            point.auxiliaryVel = 0; // advanced teams can also provide the target velocity
            point.auxiliaryArbFeedFwd = 0; // good place for kS, kV, kA, etc...

            point.profileSlotSelect0 = Constants.kPrimaryPIDSlot; /* which set of gains would you like to use [0,3]? */
            point.profileSlotSelect1 = Constants.kAuxPIDSlot; /* auxiliary PID [0,1], leave zero */
            point.zeroPos = false; /* don't reset sensor, this is done elsewhere since we have multiple sensors */
            point.isLastPoint = ((i + 1) == totalCnt); /* set this to true on the last point */
            point.useAuxPID = true; /* tell MPB that we are using both pids */

            _bufferedStream.Write(point);
        }
    }

    void ZeroAllSensors() {
        /* individuall clear the integrated sensor value of each side */
        _leftAuxFollower.getSensorCollection().setIntegratedSensorPosition(0, 100);
        _rightMaster.getSensorCollection().setIntegratedSensorPosition(0, 100);
        _pidgy.setYaw(0);
    }

	/** 
	 * Determines if SensorSum or SensorDiff should be used 
	 * for combining left/right sensors into Robot Distance.  
	 * 
	 * Assumes Aux Position is set as Remote Sensor 0.  
	 * 
	 * configAllSettings must still be called on the master config
	 * after this function modifies the config values. 
	 * 
	 * @param masterInvertType Invert of the Master Talon
	 * @param masterConfig Configuration object to fill
	 */
	 void setRobotDistanceConfigs(TalonFXInvertType masterInvertType, TalonFXConfiguration masterConfig){
		/**
		 * Determine if we need a Sum or Difference.
		 * 
		 * The auxiliary Talon FX will always be positive
		 * in the forward direction because it's a selected sensor
		 * over the CAN bus.
		 * 
		 * The master's native integrated sensor may not always be positive when forward because
		 * sensor phase is only applied to *Selected Sensors*, not native
		 * sensor sources.  And we need the native to be combined with the 
		 * aux (other side's) distance into a single robot distance.
		 */

		/* THIS FUNCTION should not need to be modified. 
		   This setup will work regardless of whether the master
		   is on the Right or Left side since it only deals with
		   distance magnitude.  */

		/* Check if we're inverted */
		if (masterInvertType == TalonFXInvertType.Clockwise){
			/* 
				If master is inverted, that means the integrated sensor
				will be negative in the forward direction.

				If master is inverted, the final sum/diff result will also be inverted.
				This is how Talon FX corrects the sensor phase when inverting 
				the motor direction.  This inversion applies to the *Selected Sensor*,
				not the native value.

				Will a sensor sum or difference give us a positive total magnitude?

				Remember the Master is one side of your drivetrain distance and 
				Auxiliary is the other side's distance.

					Phase | Term 0   |   Term 1  | Result
				Sum:  -1 *((-)Master + (+)Aux   )| NOT OK, will cancel each other out
				Diff: -1 *((-)Master - (+)Aux   )| OK - This is what we want, magnitude will be correct and positive.
				Diff: -1 *((+)Aux    - (-)Master)| NOT OK, magnitude will be correct but negative
			*/

			masterConfig.diff0Term = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(); //Local Integrated Sensor
			masterConfig.diff1Term = TalonFXFeedbackDevice.RemoteSensor0.toFeedbackDevice();   //Aux Selected Sensor
			masterConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.SensorDifference.toFeedbackDevice(); //Diff0 - Diff1
		} else {
			/* Master is not inverted, both sides are positive so we can sum them. */
			masterConfig.sum0Term = TalonFXFeedbackDevice.RemoteSensor0.toFeedbackDevice();    //Aux Selected Sensor
			masterConfig.sum1Term = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(); //Local IntegratedSensor
			masterConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.SensorSum.toFeedbackDevice(); //Sum0 + Sum1
		}

		/* Since the Distance is the sum of the two sides, divide by 2 so the total isn't double
		   the real-world value */
		masterConfig.primaryPID.selectedFeedbackCoefficient = 0.5;
	 }
}
