/**
 * Example showing the use of a raw PWM object and a PWMSpeedController object.
 * Either object can be constructed to use Pins 4,6,7,8, or 9 on Port 3 of HERO.
 * 
 * This example has two additional references under the Solution Explorer...
 * "Microsoft.SPOT.Hardware" and "Microsoft.SPOT.Hardware.PWM" 
 */
using Microsoft.SPOT;
using Microsoft.SPOT.Hardware;

namespace HERO_PWM_Example
{
	public class Program
	{
		public static void Main()
		{
			//Gamepad for input
			CTRE.Phoenix.Controller.GameController _gamepad = new CTRE.Phoenix.Controller.GameController(CTRE.Phoenix.UsbHostDevice.GetInstance());

			//simple PWM for fine control of pulse width, period, timing...
			uint period = 50000; //period between pulses
			uint duration = 1500; //duration of pulse
			PWM pwm_9 = new PWM(CTRE.HERO.IO.Port3.PWM_Pin9, period, duration, PWM.ScaleFactor.Microseconds, false);
			pwm_9.Start(); //starts the signal

			// ...and just a PWM SpeedController for motor controller (Victor SP, Talon SR, Victor 888, etc.)...
			CTRE.Phoenix.MotorControl.PWMSpeedController pwmSpeedController = new CTRE.Phoenix.MotorControl.PWMSpeedController(CTRE.HERO.IO.Port3.PWM_Pin7);

			while (true)
			{
				/* only enable motor control (PWM/CAN) if gamepad is connected.  Logitech gamepads may be disabled using the X/D switch */
				if (_gamepad.GetConnectionStatus() == CTRE.Phoenix.UsbDeviceConnection.Connected)
				{
					CTRE.Phoenix.Watchdog.Feed();
				}

				/* let axis control the pwm speed controller */
				pwmSpeedController.Set(0.10f); /* 10% */

				/* let button1 control the explicit PWM pin duration*/
				if (_gamepad.GetButton(1) == true)
				{
					pwm_9.Duration = 2000; /* 2.0ms */
				}
				else
				{
					pwm_9.Duration = 1000; /* 1.0ms */
				}

				/* yield for a bit, this controls this task's frequency */
				System.Threading.Thread.Sleep(10);
			}
		}
	}
}
