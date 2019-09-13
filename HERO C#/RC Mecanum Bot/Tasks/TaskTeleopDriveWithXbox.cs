/**
 * Read XBox game controller and update drivetrain.
 * Because XBOX game controller has enough sticks, allow strafe control via Mecanum wheels.
 */
using HERO_Mecanum_Drive_Example.Platform;

namespace HERO_Mecanum_Drive_Example
{
    public class TaskTeleopDriveWithXbox : CTRE.Phoenix.Tasking.ILoopable
    {
        public bool IsDone()
        {
            return false;
        }

        public void OnLoop()
        {
            float x = Hardware.gamepad.GetAxis(0);      // Ensure Positive is strafe-right, negative is strafe-left
            float y = +1 * Hardware.gamepad.GetAxis(1); // Ensure Positive is forward, negative is reverse
            float turn = Hardware.gamepad.GetAxis(2);  // Ensure Positive is turn-right, negative is turn-left

            CTRE.Phoenix.Util.Deadband(ref x);
            CTRE.Phoenix.Util.Deadband(ref y);
            CTRE.Phoenix.Util.Deadband(ref turn);

            if (Tasks.LowBatteryDetect.BatteryIsLow)
            {
                x *= 0.25f;
                y *= 0.25f;
                turn *= 0.25f;
            }

            Hardware.drivetrain.Set(CTRE.Phoenix.Drive.Styles.BasicStyle.PercentOutput, y, x, turn);
        }

        public void OnStart()
        {
        }

        public void OnStop()
        {
        }
    }
}
