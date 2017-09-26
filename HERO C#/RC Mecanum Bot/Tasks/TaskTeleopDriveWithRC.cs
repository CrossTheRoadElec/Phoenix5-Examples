/**
 * Reads Futaba RC radio controller and update drivetrain.
 * When Futaba ch3 switch is enabled, robot will hold heading with Pigeon IMU.
 */
using HERO_Mecanum_Drive_Example.Platform;

namespace HERO_Mecanum_Drive_Example
{
    public class TaskTeleopDriveWithRC : CTRE.Tasking.ILoopable
    {
        private float[] _pulseWidUs = new float[4];
        private float[] _periodUs = new float[4];

        private bool _holdHeading = false;

        public bool IsDone()
        {
            return false;
        }

        public void OnLoop()
        {
            /* gains */
            Hardware.ServoHoldHeading.ServoParameters.P = 0.01f;
            Hardware.ServoHoldHeading.ServoParameters.D = 0.001f;

            /* poll RC radio */
            float forward = Hardware.Futaba3Ch.GetDutyCyclePerc(CTRE.RCRadio3Ch.Channel.Channel2);
            float turn = Hardware.Futaba3Ch.GetDutyCyclePerc(CTRE.RCRadio3Ch.Channel.Channel1);
            float strafe = 0;

            bool toggleSwitch = Hardware.Futaba3Ch.GetSwitchValue(CTRE.RCRadio3Ch.Channel.Channel3);

            CTRE.Util.Deadband(ref forward);
            CTRE.Util.Deadband(ref turn);
            CTRE.Util.Deadband(ref strafe);

            if (Tasks.LowBatteryDetect.BatteryIsLow)
            {
                forward *= 0.25f;
                turn *= 0.25f;
                strafe *= 0.25f;
            }

            if (toggleSwitch && (turn == 0) )
            {
                Hardware.ServoHoldHeading.Set(CTRE.Drive.Styles.Basic.PercentOutput, forward, strafe);
            }
            else
            {
                Hardware.ServoHoldHeading.Disable(); /* let the servo know that we've stopped using it */
                Hardware.drivetrain.Set(CTRE.Drive.Styles.Basic.PercentOutput, forward, strafe, turn);
            }
        }

        public void OnStart()
        {
        }

        public void OnStop()
        {
        }
    }
}
