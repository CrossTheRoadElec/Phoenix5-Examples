using System;
using System.Threading;
using Microsoft.SPOT;
using System.Text;

using CTRE.Phoenix;
using CTRE.Phoenix.Controller;
using CTRE.Phoenix.MotorControl;
using CTRE.Phoenix.MotorControl.CAN;

/**
 * Description:
 * The VelocityClosedLoop example demonstrates the velocity closed-loop servo.
 * Tested with Logitech F710 USB Gamepad inserted into HERO USB A port.
 * 
 * Be sure to select the correct feedback sensor using ConfigSelectedFeedbackSensor() below.
 * Use Percent Output Mode (Using Left Joystick) to confirm talon is driving 
 * forward (Green LED on Talon/Victor) when the postion sensor is moving in the postive 
 * direction. If this is not the case, flip the boolean input in SetSensorPhase().
 * 
 * Controls:
 * Button 1 (X): When held, start and run Velocity Closed Loop on Talon/Victor
 * Left Joystick Y-Axis:
 * 	+ Percent Output: Throttle Talon forward and reverse, use to confirm hardware setup
 * 	+ Velocity Closed Loop: Servo Talon forward and reverse [-2000, 2000] RPM
 * 
 * Gains for Velocity Closed Loop may need to be adjusted in Constants
 * 
 * Supported Version:
 * - Talon SRX: 11.11
 */

namespace HERO_Velocity_Control_Example
{
	public class Program
	{
		

		public static void Main()
		{
			/* Hardware */
			TalonSRX _talon = new TalonSRX(1);

			/** Use a USB gamepad plugged into the HERO */
			GameController _gamepad = new GameController(UsbHostDevice.GetInstance());

			/* String for output */
			StringBuilder _sb = new StringBuilder();

			/** hold bottom left shoulder button to enable motors */
			const uint kEnableButton = 7;

			/* Loop tracker for prints */
			int _loops = 0;

			/* Initialization */
			/* Factory Default all hardware to prevent unexpected behaviour */
			_talon.ConfigFactoryDefault();

			/* Config sensor used for Primary PID [Velocity] */
			_talon.ConfigSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
												Constants.kPIDLoopIdx,
												Constants.kTimeoutMs);

			/**
			 * Phase sensor accordingly. 
			 * Positive Sensor Reading should match Green (blinking) Leds on Talon
			 */
			_talon.SetSensorPhase(false);

			/* Config the peak and nominal outputs */
			_talon.ConfigNominalOutputForward(0, Constants.kTimeoutMs);
			_talon.ConfigNominalOutputReverse(0, Constants.kTimeoutMs);
			_talon.ConfigPeakOutputForward(1, Constants.kTimeoutMs);
			_talon.ConfigPeakOutputReverse(-1, Constants.kTimeoutMs);

			/* Config the Velocity closed loop gains in slot0 */
			_talon.Config_kF(Constants.kPIDLoopIdx, Constants.kF, Constants.kTimeoutMs);
			_talon.Config_kP(Constants.kPIDLoopIdx, Constants.kP, Constants.kTimeoutMs);
			_talon.Config_kI(Constants.kPIDLoopIdx, Constants.kI, Constants.kTimeoutMs);
			_talon.Config_kD(Constants.kPIDLoopIdx, Constants.kD, Constants.kTimeoutMs);

			/* loop forever */
			while (true)
			{
				/* Get gamepad axis */
				double leftYstick = -1 * _gamepad.GetAxis(1);

				/* Get Talon/Victor's current output percentage */
				double motorOutput = _talon.GetMotorOutputPercent();

				/* Prepare line to print */
				_sb.Append("\tout:");
				/* Cast to int to remove decimal places */
				_sb.Append((int)(motorOutput * 100));
				_sb.Append("%");    // Percent

				_sb.Append("\tspd:");
				_sb.Append(_talon.GetSelectedSensorVelocity(Constants.kPIDLoopIdx));
				_sb.Append("u");    // Native units

				/** 
				 * When button 1 is held, start and run Velocity Closed loop.
				 * Velocity Closed Loop is controlled by joystick position x2000 RPM, [-2000, 2000] RPM
				 */
				if (_gamepad.GetButton(1))
				{
					/* Velocity Closed Loop */

					/**
					 * Convert 2000 RPM to units / 100ms.
					 * 4096 Units/Rev * 2000 RPM / 600 100ms/min in either direction:
					 * velocity setpoint is in units/100ms
					 */
					double targetVelocity_UnitsPer100ms = leftYstick * 2000.0 * 4096 / 600;
					/* 2000 RPM in either direction */
					_talon.Set(ControlMode.Velocity, targetVelocity_UnitsPer100ms);

					/* Append more signals to print when in speed mode. */
					_sb.Append("\terr:");
					_sb.Append(_talon.GetClosedLoopError(Constants.kPIDLoopIdx));
					_sb.Append("\ttrg:");
					_sb.Append(targetVelocity_UnitsPer100ms);
				}
				else
				{
					/* Percent Output */

					_talon.Set(ControlMode.PercentOutput, leftYstick);
				}

				/* Print built string every 10 loops */
				if (++_loops >= 10)
				{
					_loops = 0;
					Debug.Print(_sb.ToString());
				}
				/* Reset built string */
				_sb.Clear();

				//if (_gamepad.GetConnectionStatus() == CTRE.UsbDeviceConnection.Connected) // check if gamepad is plugged in OR....
				if (_gamepad.GetButton(kEnableButton)) // check if bottom left shoulder buttom is held down.
				{
					/* then enable motor outputs*/
					Watchdog.Feed();
				}

				/* wait a bit */
				System.Threading.Thread.Sleep(20);
			}
		}
	}

	static class Constants
	{
		/**
		 * Which PID slot to pull gains from. Starting 2018, you can choose from
		 * 0,1,2 or 3. Only the first two (0,1) are visible in web-based
		 * configuration.
		 */
		public const int kSlotIdx = 0;

		/**
		 * Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops. For
		 * now we just want the primary one.
		 */
		public const int kPIDLoopIdx = 0;

		/**
		 * Set to zero to skip waiting for confirmation, set to nonzero to wait and
		 * report to debug print if action fails.
		 */
		public const int kTimeoutMs = 30;

		/**
		 * PID Gains may have to be adjusted based on the responsiveness of control loop.
		 * kF: 1023 x 0.50 represents output value to Talon at 50%, 53000 represents Velocity units at 50% output
		 */
		public const float kP = 0.005f;
		public const float kI = 0f;
		public const float kD = 0;
		public const float kF = (1023f * 0.50f) / 53000f;
		public const float IZone = 0;
	}
}
