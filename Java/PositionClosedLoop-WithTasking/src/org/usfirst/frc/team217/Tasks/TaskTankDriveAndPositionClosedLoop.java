package org.usfirst.frc.team217.Tasks;

import org.usfirst.frc.team217.robot.Platform.Hardware;

import com.ctre.phoenix.ILoopable;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteLimitSwitchSource;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

public class TaskTankDriveAndPositionClosedLoop implements ILoopable {
	
	/* Some locals for tracking states */
	boolean lastButton1 = false;
	boolean lastButton3 = false;
	boolean operationState = false;
	
	public void onStart() {
		/* Initialize Talons and Victors */

		/* Duration of time it takes to go from neutral to full throttle */
		Hardware.leftMaster.configOpenloopRamp(0.5, 0);
		Hardware.leftFollower.configOpenloopRamp(0, 0);
		Hardware.rightMaster.configOpenloopRamp(0.5, 0);
		Hardware.rightFollower.configOpenloopRamp(0, 0);

		/* Invert right side of drivetrain (Based on which direction you want forward */
		Hardware.leftMaster.setInverted(false);
		Hardware.leftFollower.setInverted(false);
		Hardware.rightMaster.setInverted(true);
		Hardware.rightFollower.setInverted(true);

		/* Set Neutral mode of all motor controllers to brake or coast */
		Hardware.leftMaster.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Brake);
		Hardware.leftFollower.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Brake);
		Hardware.rightMaster.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Brake);
		Hardware.rightFollower.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Brake);
	

		/* Set Victor to follow right master, set Talon to follow left master */
		Hardware.rightFollower.follow(Hardware.rightMaster);
		Hardware.leftFollower.set(com.ctre.phoenix.motorcontrol.ControlMode.Follower, 6);
		
		/* Sensor configuration */
		Hardware.rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		Hardware.rightMaster.setSensorPhase(true);
		Hardware.leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		Hardware.leftMaster.setSensorPhase(true);
		
		/* Soft limits, configured to ensure they are disabled */
		Hardware.rightMaster.configForwardSoftLimitThreshold(10000, 0);
		Hardware.rightMaster.configReverseSoftLimitThreshold(-10000, 0);
		Hardware.rightMaster.configForwardSoftLimitEnable(false, 0);
		Hardware.rightMaster.configReverseSoftLimitEnable(false, 0);
		
		Hardware.leftMaster.configForwardSoftLimitThreshold(10000, 0);
		Hardware.leftMaster.configReverseSoftLimitThreshold(-10000, 0);
		Hardware.leftMaster.configForwardSoftLimitEnable(false, 0);
		Hardware.leftMaster.configReverseSoftLimitEnable(false, 0);
		
		/* Values for both sets of gains */
		double kp = 1;
		double kd = 75;
		double ki = 0;
		double kf = 0;
		int gain = 0;
		
		/* Set profile and value, here we have selected profile 0 */
		Hardware.rightMaster.config_kP(0, kp, 0);
		Hardware.rightMaster.config_kI(0, ki, 0);
		Hardware.rightMaster.config_kD(0, kd, 0);
		Hardware.rightMaster.config_kF(0, kf, 0);
		Hardware.rightMaster.config_IntegralZone(0, gain, 0);
		Hardware.rightMaster.configAllowableClosedloopError(0, 50, 0);
		
		Hardware.leftMaster.config_kP(0, kp, 0);
		Hardware.leftMaster.config_kI(0, ki, 0);
		Hardware.leftMaster.config_kD(0, kd, 0);
		Hardware.leftMaster.config_kF(0, kf, 0);
		Hardware.leftMaster.config_IntegralZone(0, gain, 0);
		Hardware.leftMaster.configAllowableClosedloopError(0, 50, 0);

		/* Configure the maximum output to both closed loop and open loop control */
		Hardware.rightMaster.configPeakOutputForward(0.3, 0);
		Hardware.rightMaster.configPeakOutputReverse(-0.3, 0);

		Hardware.leftMaster.configPeakOutputForward(0.3, 0);
		Hardware.leftMaster.configPeakOutputReverse(-0.3, 0);
	}

	public void onStop() {
		/* Don't ever stop */
	}

	public boolean isDone() {
		/* Task is never done */
		return false;
	}

	public void onLoop(){
		/* Grab Gamepad/Joystick values */
		float forward = -1 * (float) Hardware.gamepad.getRawAxis(1);
		float turn = (float) Hardware.gamepad.getRawAxis(2);
		forward = Deadband(forward, 0.10f);
		turn = Deadband(turn, 0.10f);
		turn *= 0.25f;

		/* Perform math to determine outputs of each side */
		float leftOutput = (forward + turn);
		float rightOutput = (forward - turn);
		
		/* Changes states and reset position with button presses */
		boolean button1 = Hardware.gamepad.getRawButton(1);
		boolean button3 = Hardware.gamepad.getRawButton(3);	
		if(button1 && !lastButton1){
			operationState = true;
			Hardware.rightMaster.setSelectedSensorPosition(0, 0, 0);
			Hardware.leftMaster.setSelectedSensorPosition(0, 0, 0);
		}
		else if(button3 && !lastButton3)
			operationState = false;	
		lastButton1 = button1;
		lastButton3 = button3;
		
		/* False = PercentOutput, True = PositionClosedLoop */
		if(operationState == false){
			/* Enter percent-output open-loop mode */
			Hardware.leftMaster.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, leftOutput);
			Hardware.rightMaster.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, rightOutput);
		}else{
			/* Enter position closed-loop mode */
			/* Currently using CTRE Mag Encoder (relative), 4096 ticks per rotation */
			
			/* Select PID slot for profile */
			float targetDistanceToDrive = forward * 4096 * 5;	//5 rotations
			Hardware.rightMaster.selectProfileSlot(0, 0);
			Hardware.leftMaster.selectProfileSlot(0, 0);
			Hardware.rightMaster.set(ControlMode.Position, targetDistanceToDrive);
			Hardware.leftMaster.set(ControlMode.Position, targetDistanceToDrive);
		}
	}
	
	/* Deadbands a value between [-1, 1] */
	public float Deadband(float value, float Percentage) {
		if (value < -Percentage) 
		{
			/* Nothing */
		} 
		else if (value > +Percentage) 
		{
			/* Nothing */
		} 
		else 
		{
			/* Something */
			value = 0;
		}
		return value;
	}
}
