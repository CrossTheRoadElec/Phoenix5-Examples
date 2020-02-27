/** 
 * Simple class containing constants used throughout project
 */
package frc.robot.Platform;

import com.ctre.phoenix.CANifier;

public class Constants {
	public final static float GamepadDeadband = 0.03f;

	public final static int GamePadAxis_x = 0;
	public final static int GamePadAxis_y = 1;
	public final static int GamePadAxis_red = 0;
	public final static int GamePadAxis_green = 1;
	public final static int GamePadAxis_blue = 5;

	public final static CANifier.PWMChannel kMotorControllerCh = CANifier.PWMChannel.PWMChannel2;
}
