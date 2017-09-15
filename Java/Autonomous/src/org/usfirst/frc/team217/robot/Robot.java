
package org.usfirst.frc.team217.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;

import com.ctre.CTRLogger;
import com.ctre.MotorControl.*;
import com.ctre.PigeonImu;
import com.ctre.Drive.SensoredTank;
import com.ctre.Drive.Styles.Basic;
import com.ctre.Drive.Styles.Smart;
import com.ctre.Motion.*;
import com.ctre.Mechanical.*;
import com.ctre.Schedulers.*;
import com.ctre.Time.StopWatch;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	CANTalon left1 = new CANTalon(3);
	CANTalon left2 = new CANTalon(2);
	CANTalon right1 = new CANTalon(6);
	CANTalon right2 = new CANTalon(4);
	
	SensoredGearbox left = new SensoredGearbox(4096, left1, left2, SmartMotorController.FeedbackDevice.CtreMagEncoder_Relative);
	SensoredGearbox right = new SensoredGearbox(4096, right1, right2, SmartMotorController.FeedbackDevice.CtreMagEncoder_Relative);
	
	SensoredTank drive = new SensoredTank(left, right, false, true, 2, 28.5f);
	
	PigeonImu p = new PigeonImu(right2);
	
	Joystick j = new Joystick(0);
	
	SequentialScheduler auton = new SequentialScheduler(10);
	
/////////////////////////////////////////////////////////////////////////////////////
	ServoParameters drivingStraight = new ServoParameters();
	ServoParameters drivingForward = new ServoParameters();
	ServoParameters turning = new ServoParameters();
	
/////////////////////////////////////////////////////////////////////////////////////	
ServoStraightDistanceWithImu firstSegment = new ServoStraightDistanceWithImu(p, drive, Smart.PercentOutput, drivingStraight, drivingForward, 0f, 50f);
ServoZeroTurnWithImu turnToAirship = new ServoZeroTurnWithImu(p, drive, Basic.PercentOutput, 30, turning);
ServoStraightDistanceWithImu toAirship = new ServoStraightDistanceWithImu(p, drive, Smart.PercentOutput, drivingStraight, drivingForward, 30f, 55);
ServoStraightDistanceWithImu backUp = new ServoStraightDistanceWithImu(p, drive, Smart.PercentOutput, drivingStraight, drivingForward, 30, -20);
	
	@Override
	public void robotInit()
	{
		CTRLogger.Open();
		
		firstSegment.SetStraightServoParams(drivingStraight);
		firstSegment.SetDistanceServoParams(drivingForward);
		turnToAirship.SetServoParams(turning);
		toAirship.SetStraightServoParams(drivingStraight);
		toAirship.SetDistanceServoParams(drivingForward);
		
		
		auton.Add(firstSegment);
		auton.Add(turnToAirship);
		auton.Add(toAirship);
		auton.Add(backUp);
	}
	
	@Override
	public void teleopPeriodic()
	{
		drive.set(Basic.PercentOutput, -(float)j.getRawAxis(1), (float)j.getRawAxis(4));
		System.out.println(drive.GetDistance() + " " + left1.getPosition() + " " + right1.getPosition());
		
	}
	
	StopWatch s = new StopWatch();
	@Override
	public void autonomousPeriodic()
	{
		while(true)
		{
			System.out.println(s.getDurationMs());
			s.start();
			auton.Process();
			if(auton.Iterated())drive.SetPosition(0);
			while(s.getDurationMs() < 10) {}
			if(j.getRawButton(1)) break;
		}
	}

	@Override
	public void autonomousInit()
	{
		drive.SetPosition(0);
		p.SetYaw(0);
		left1.reverseSensor(true);
		
		drivingStraight.P = .03f;
		drivingStraight.I = 0.002f;
		drivingStraight.IZone = 1f;
		drivingStraight.IMax = 0.12f;
		drivingStraight.D = 0.00f;
		drivingStraight.maxOut = 0.15f;
		drivingStraight.timeToDone = 0.2f;
		drivingStraight.allowedError = 0.5f;
		
		drivingForward.P = .05f;
		drivingForward.I = 0.001f;
		drivingForward.IZone = 2f;
		drivingForward.IMax = 0.1f;
		drivingForward.D = 0.04f;
		drivingForward.maxOut = 0.5f;
		drivingForward.timeToDone = 0.2f;
		drivingForward.allowedError = 0.5f;
		
		turning.P = .05f;
		turning.I = .004f;
		turning.IZone = 4f;
		turning.IMax = 1f;
		turning.D = .003f;
		turning.maxOut = .2f;
		turning.timeToDone = 0.2f;
		turning.allowedError = 0.5f;
		
		auton.Start();
	}
	
	@Override
	public void teleopInit()
	{
		left1.reverseSensor(true);
	}
}
