using System;
using System.Threading;
using CTRE.Phoenix.Controller;
using CTRE.Phoenix.MotorControl;
using CTRE.Phoenix.MotorControl.CAN;

namespace HERO_FRC_Control
{
	public class Program
	{
		public static void Main()
		{
			/* Get the Joystick values from the CANbus.  */
			GameController _gamepad = new GameController(new CANJoystickSource(0));
			
			TalonSRX leftTalon = new TalonSRX(10);
			TalonSRX rightTalon = new TalonSRX(11);

			/* Set Inverted on one side so "forward" relative to the robot is a Green LED state */
			leftTalon.SetInverted(false);
			rightTalon.SetInverted(true);

			/* loop forever */
			while (true)
			{
				/* Basic Drivetrain, negate the axes so forward is positive */
				leftTalon.Set(ControlMode.PercentOutput, _gamepad.GetAxis(1) * -1);
				rightTalon.Set(ControlMode.PercentOutput, _gamepad.GetAxis(3) * -1);

				/* wait a bit */
				Thread.Sleep(10);

				/* Normally we would need to feed the Watchdog here.
				 * We're relying on the roboRIO to provide the enable
				 * signal, so it's not necessary.
				 **/
			}
		}
	}
}