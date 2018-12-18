/** 
 * Simple class containing hardware used throughout project
 */
package frc.robot.Platform;

import edu.wpi.first.wpilibj.Joystick;
import com.ctre.phoenix.CANifier;

public class Hardware {
	public static CANifier canifier = new CANifier(0);
	public static Joystick gamepad = new Joystick(0);
}
