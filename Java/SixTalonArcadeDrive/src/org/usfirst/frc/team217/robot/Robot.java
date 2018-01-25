package org.usfirst.frc.team217.robot;

import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Robot extends IterativeRobot {

	/* talons for arcade drive */
	WPI_TalonSRX _frontLeftMotor = new WPI_TalonSRX(6);
	WPI_TalonSRX _frontRightMotor = new WPI_TalonSRX(2);

	/* extra talons and victors for six motor drives */
	WPI_TalonSRX _leftSlave1 = new WPI_TalonSRX(5);
	WPI_VictorSPX _rightSlave1 = new WPI_VictorSPX(7);
	WPI_TalonSRX _leftSlave2 = new WPI_TalonSRX(4);
	WPI_VictorSPX _rightSlave2 = new WPI_VictorSPX(17);

	DifferentialDrive _drive = new DifferentialDrive(_frontLeftMotor, _frontRightMotor);

	Joystick _joy = new Joystick(0);

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	public void robotInit() {
		/*
		 * take our extra talons and just have them follow the Talons updated in
		 * arcadeDrive
		 */
		_leftSlave1.follow(_frontLeftMotor);
		_leftSlave2.follow(_frontLeftMotor);
		_rightSlave1.follow(_frontRightMotor);
		_rightSlave2.follow(_frontRightMotor);

		/* drive robot forward and make sure all 
		 * motors spin the correct way.
		 * Toggle booleans accordingly.... */
		_frontLeftMotor.setInverted(false);
		_leftSlave1.setInverted(false);
		_leftSlave2.setInverted(false);
		
		_frontRightMotor.setInverted(false);
		_rightSlave1.setInverted(false);
		_rightSlave2.setInverted(false);
	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
		/* sign this so forward is positive */
		double forward = -1.0 * _joy.getY();
		/* sign this so right is positive. */
		double turn = +1.0 * _joy.getZ();
		/* deadband */
		if (Math.abs(forward) < 0.10) {
			/* within 10% joystick, make it zero */
			forward = 0;
		}
		if (Math.abs(turn) < 0.10) {
			/* within 10% joystick, make it zero */
			turn = 0;
		}
		/* print the joystick values to sign them, comment
		 * out this line after checking the joystick directions. */
		System.out.println("JoyY:" + forward + "  turn:" + turn );
		/* drive the robot, when driving forward one side will be red.  
		 * This is because DifferentialDrive assumes 
		 * one side must be negative */
		_drive.arcadeDrive(forward, turn);
	}
}
