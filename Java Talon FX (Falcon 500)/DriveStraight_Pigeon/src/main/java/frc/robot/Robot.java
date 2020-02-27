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
 * The DriveStraight_Pigeon example demonstrates the ability to get telemetry on orientation 
 * from Pigeon and perform Straight Drive Servo-ing. This is done by providing more/less output
 * on either the left or right side of the robot based on the error between desired yaw and 
 * current yaw. The project is basic PID Controller done in the roboRIO on Pigeon Yaw.
 *
 * If Pigeon is present on CANbus, or ribbon-cabled to a CAN-Talon, the robot will use the IMU to servo.
 * If Pigeon is not present, robot will simply apply the same throttle to both sides.
 * 
 * NOTE:
 * This is different from DriveStraight_AuxPigeon in that this project does not use 
 * the auxiliary feature, therfore performing the Closed Loop on Yaw in the RIO rather
 * than in the the motor controller. 
 * 
 * Controls:
 * Button 5: When held, start and run Pigeon straight drive, holding the angle the robot was 
 * 	in when the button was initally pressed during the entire hold duration.
 * Left Joystick Y-Axis: Drive robot forward and reverse.
 * Right Joystick X-Axis: 
 * 	+ Normal mode: When Button 5 is released, turn robot left and right.
 * 	+ Pigeon Straight Drive: When Button 5 is held, do nothing. Maintain current heading/yaw
 * 
 * When developing robot applications with IMUs, it's important to design in what happens if
 * the IMU is disconnected or un-powered.
 * 
 * Supported Version:
 * 	- Talon FX: 20.2.3.0
 *  - Pigeon IMU: 20.0
 */
package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {
    /* Hardware */
	TalonFX _leftFront;    // Drivetrain
	TalonFX _rightFront;  // Drivetrain
	TalonFX _leftRear;     // Drivetrain
	TalonFX _rightRear;    // Drivetrain
    PigeonIMU _pidgey;      // Pigeon IMU used to enforce straight drive
	Joystick _driveStick;	// Joystick object on USB port 1

	/** States for tracking whats controlling the drivetrain */
	enum GoStraight
	{
        Off, 
        UsePigeon, 
        SameThrottle
	};
	GoStraight _goStraight = GoStraight.Off;    // Start example with GoStraight Off

	/**
	 * Some gains for heading servo
	 */
	double kPgain = 0.04; 				// percent throttle per degree of error */
	double kDgain = 0.0004; 			// percent throttle per angular velocity dps */
	double kMaxCorrectionRatio = 0.30;	// cap corrective turning throttle to 30 percent of forward throttle

	/** Holds the current angle to servo to */
	double _targetAngle = 0;
	
	/** Count loops to print every second or so */
	int _printLoops = 0;

	public Robot() {
        /* Init Hardware */
		_leftFront = new TalonFX(1);
		_rightFront = new TalonFX(2);
		_leftRear = new TalonFX(3);
		_rightRear = new TalonFX(2);
		_pidgey = new PigeonIMU(3);             // Change ID accordingly 
        
		/* Define joystick being used at USB port #0 on the Drivers Station */
		_driveStick = new Joystick(0);	
	}
	
    public void teleopInit() {
        /* Factory Default all Hardware to prevent unexpected behaviour */
        _rightFront.configFactoryDefault();
        _leftFront.configFactoryDefault();
        _rightRear.configFactoryDefault();
        _leftRear.configFactoryDefault();
        _pidgey.configFactoryDefault();
		/*
		 * Talon FX does not need sensor phase set for its integrated sensor
		 * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
		 * and the user calls getSelectedSensor* to get the sensor's position/velocity.
		 * 
		 * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
		 */
        // _rightFront.setSensorPhase(true);
        // _leftFront.setSensorPhase(true);

        /* nonzero to block the config until success, zero to skip checking */
        final int kTimeoutMs = 30;
    
        /* reset heading, angle measurement wraps at plus/minus 23,040 degrees (64 rotations) */
    	_pidgey.setFusedHeading(0.0, kTimeoutMs);
		_goStraight = GoStraight.Off;    //Start example with GoStraight Off
    }
	
    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
    	/* get Pigeon status information from Pigeon API */
		PigeonIMU.GeneralStatus genStatus = new PigeonIMU.GeneralStatus();
		PigeonIMU.FusionStatus fusionStatus = new PigeonIMU.FusionStatus();
		double [] xyz_dps = new double [3];
		/* grab some input data from Pigeon and gamepad*/
		_pidgey.getGeneralStatus(genStatus);
		_pidgey.getRawGyro(xyz_dps);
		_pidgey.getFusedHeading(fusionStatus);
        double currentAngle = fusionStatus.heading;
		boolean angleIsGood = (_pidgey.getState() == PigeonIMU.PigeonState.Ready) ? true : false;
		double currentAngularRate = xyz_dps[2];
		/* get input from gamepad */
		boolean userWantsGoStraight = _driveStick.getRawButton(5); /* top left shoulder button */
		double forwardThrottle = _driveStick.getY() * -1.0; /* sign so that positive is forward */
		double turnThrottle = _driveStick.getTwist() * -0.5;/* sign so that positive means turn left */
		/* deadbands so centering joysticks always results in zero output */
		forwardThrottle = Deadband(forwardThrottle);
        turnThrottle = Deadband(turnThrottle);
        
		/* state machine to update our goStraight selection */
		switch (_goStraight) {
			/* go straight is off, better check gamepad to see if we should enable the feature */
			case Off:
				if (userWantsGoStraight == false) {
					/* nothing to do */
				} else if (angleIsGood == false) {
					/* user wants to servo but Pigeon isn't connected? */
					_goStraight = GoStraight.SameThrottle; /* just apply same throttle to both sides */
				} else {
					/* user wants to servo, save the current heading so we know where to servo to. */
					_goStraight = GoStraight.UsePigeon;
					_targetAngle = currentAngle;
				}
                break;
                
			/* we are servo-ing heading with Pigeon */
			case UsePigeon:
				if (userWantsGoStraight == false) {
					_goStraight = GoStraight.Off; /* user let go, turn off the feature */
				} else if (angleIsGood == false) {
					_goStraight = GoStraight.SameThrottle; /* we were servo-ing with pidgy, but we lost connection?  Check wiring and deviceID setup */
				} else {
					/* user still wants to drive straight, keep doing it */
				}
				break;

			/* we are simply applying the same throttle to both sides, apparently Pigeon is not connected */
			case SameThrottle:
				if (userWantsGoStraight == false) {
					_goStraight = GoStraight.Off; /* user let go, turn off the feature */
				} else {
					/* user still wants to drive straight, keep doing it */
				}
				break;
		}

		/* if we can servo with IMU, do the math here */
		if (_goStraight == GoStraight.UsePigeon) {
			/* very simple Proportional and Derivative (PD) loop with a cap,
			 * replace with favorite close loop strategy or leverage future Talon <=> Pigeon features. */
			turnThrottle = (_targetAngle - currentAngle) * kPgain - (currentAngularRate) * kDgain;
			/* the max correction is the forward throttle times a scalar,
			 * This can be done a number of ways but basically only apply small turning correction when we are moving slow
			 * and larger correction the faster we move.  Otherwise you may need stiffer pgain at higher velocities. */
			double maxThrot = MaxCorrection(forwardThrottle, kMaxCorrectionRatio);
			turnThrottle = Cap(turnThrottle, maxThrot);
		} else if (_goStraight == GoStraight.SameThrottle) {
			/* clear the turn throttle, just apply same throttle to both sides */
			turnThrottle = 0;
		} else {
			/* do nothing */
		}

		/* positive turnThrottle means turn to the left, this can be replaced with ArcadeDrive object, or teams drivetrain object */
		double left = forwardThrottle - turnThrottle;
		double right = forwardThrottle + turnThrottle;
		left = Cap(left, 1.0);
		right = Cap(right, 1.0);

		/* our right side motors need to drive negative to move robot forward */
		_leftFront.set(TalonFXControlMode.PercentOutput, left);
		_leftRear.set(TalonFXControlMode.PercentOutput, left);
		_rightFront.set(TalonFXControlMode.PercentOutput, -1. * right);
		_rightRear.set(TalonFXControlMode.PercentOutput, -1. * right);

		/* Prints for debugging */
		if (++_printLoops > 50){
			_printLoops = 0;
			/* Create Print Block */
			System.out.println("------------------------------------------");
			System.out.println("error: " + (_targetAngle - currentAngle) );
			System.out.println("angle: "+ currentAngle);
			System.out.println("rate: "+ currentAngularRate);
			System.out.println("noMotionBiasCount: "+ genStatus.noMotionBiasCount);
			System.out.println("tempCompensationCount: "+ genStatus.tempCompensationCount);
			System.out.println( angleIsGood ? "Angle is good" : "Angle is NOT GOOD");
			System.out.println("------------------------------------------");
		}
    }

    /** 
     * @param axisVal to deadband.
     * @return 10% deadbanded joystick value
     */
	double Deadband(double axisVal) {
		if (axisVal < -0.10)
			return axisVal;
		if (axisVal > +0.10)
			return axisVal;
		return 0;
    }
    
	/** 
     * @param value to cap.
	 * @param peak positive double representing the maximum (peak) value.
	 * @return a capped value.
	 */
	double Cap(double value, double peak) {
		if (value < -peak)
			return -peak;
		if (value > +peak)
			return +peak;
		return value;
    }
    
	/**
	 * Given the robot forward throttle and ratio, return the max
	 * corrective turning throttle to adjust for heading.  This is
	 * a simple method of avoiding using different gains for
	 * low speed, high speed, and no-speed (zero turns).
	 */
	double MaxCorrection(double forwardThrot, double scalor) {
		/* make it positive */
		if(forwardThrot < 0) {forwardThrot = -forwardThrot;}
		/* max correction is the current forward throttle scaled down */
		forwardThrot *= scalor;
		/* ensure caller is allowed at least 10% throttle,
		 * regardless of forward throttle */
		if(forwardThrot < 0.10)
			return 0.10;
		return forwardThrot;
    }
}
