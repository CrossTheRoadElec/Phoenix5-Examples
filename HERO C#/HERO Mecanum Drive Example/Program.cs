using System;
using System.Threading;
using Microsoft.SPOT;
using System.Text;

using CTRE.Phoenix;
using CTRE.Phoenix.Controller;
using CTRE.Phoenix.MotorControl;
using CTRE.Phoenix.MotorControl.CAN;

namespace HERO_Mecanum_Drive_Example
{
    public class Program
    {
        /* create a talon */
        static TalonSRX leftFrnt = new TalonSRX(4);
        static TalonSRX leftRear = new TalonSRX(3);
        static TalonSRX rghtFrnt = new TalonSRX(2);
        static TalonSRX rghtRear = new TalonSRX(1);

		static GameController _gamepad = new GameController(UsbHostDevice.GetInstance());

        public static void Main()
        {
			/* Factory Default all hardware to prevent unexpected behaviour */
			leftFrnt.ConfigFactoryDefault();
			leftRear.ConfigFactoryDefault();
			rghtFrnt.ConfigFactoryDefault();
			rghtRear.ConfigFactoryDefault();
            /* loop forever */
            while (true)
            {
                /* keep feeding watchdog to enable motors */
                Watchdog.Feed();

                Drive();

                Thread.Sleep(20);
            }
        }
        /**
         * If value is within 10% of center, clear it.
         * @param [out] floating point value to deadband.
         */
        static void Deadband(ref float value)
        {
            if (value < -0.10)
            {
                /* outside of deadband */
            }
            else if (value > +0.10)
            {
                /* outside of deadband */
            }
            else
            {
                /* within 10% so zero it */
                value = 0;
            }
        }
        /**
         * Nomalize the vector sum of mecanum math.  Some prefer to 
         * scale from the max possible value to '1'.  Others
         * prefer to simply cut off if the sum exceeds '1'.
         */
        static void Normalize(ref float toNormalize)
        {
            if (toNormalize > 1)
            {
                toNormalize = 1;
            }
            else if (toNormalize < -1)
            {
                toNormalize = -1;
            }
            else
            {
                /* nothing to do */
            }
        }
        static void Drive()
        {
            float x = _gamepad.GetAxis(0);      // Positive is strafe-right, negative is strafe-left
            float y = -1 * _gamepad.GetAxis(1); // Positive is forward, negative is reverse
            float turn = _gamepad.GetAxis(2);  // Positive is turn-right, negative is turn-left

            Deadband(ref x);
            Deadband(ref y);
            Deadband(ref turn);

            float leftFrnt_throt = y + x + turn; // left front moves positive for forward, strafe-right, turn-right
            float leftRear_throt = y - x + turn; // left rear moves positive for forward, strafe-left, turn-right
            float rghtFrnt_throt = y - x - turn; // right front moves positive for forward, strafe-left, turn-left
            float rghtRear_throt = y + x - turn; // right rear moves positive for forward, strafe-right, turn-left

            /* normalize here, there a many way to accomplish this, this is a simple solution */
            Normalize(ref leftFrnt_throt);
            Normalize(ref leftRear_throt);
            Normalize(ref rghtFrnt_throt);
            Normalize(ref rghtRear_throt);

            /* everything up until this point assumes positive spins motor so that robot moves forward.
                But typically one side of the robot has to drive negative (red LED) to move robor forward.
                Assuming the left-side has to be negative to move robot forward, flip the left side */
            leftFrnt_throt *= -1;
            leftRear_throt *= -1;

            leftFrnt.Set(ControlMode.PercentOutput, leftFrnt_throt);
            leftRear.Set(ControlMode.PercentOutput, leftRear_throt);
            rghtFrnt.Set(ControlMode.PercentOutput, rghtFrnt_throt);
            rghtRear.Set(ControlMode.PercentOutput, rghtRear_throt);
        }
    }
}
