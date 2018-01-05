package org.usfirst.frc.team217.robot.Platform;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.Joystick;

public class Hardware {
	public static TalonSRX leftMaster = new TalonSRX(6);
	public static TalonSRX leftFollower = new TalonSRX(4);
	public static TalonSRX rightMaster = new TalonSRX(3);
	public static VictorSPX rightFollower = new VictorSPX(7);
	
	public static Joystick gamepad = new Joystick(0);
}