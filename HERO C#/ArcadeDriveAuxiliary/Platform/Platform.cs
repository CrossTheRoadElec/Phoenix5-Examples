using System;
using ArcadeDriveAuxiliary.Platform;
using CTRE.Phoenix.MotorControl.CAN;
using CTRE.Phoenix.MotorControl;
using CTRE.Phoenix.Mechanical;
using CTRE.Phoenix.Controller;
using CTRE.Phoenix.Sensors;

namespace ArcadeDriveAuxiliary.Platform
{
    public static class Constants
    {
        /* None used in project */
    }

    public static class Hardware
    {
        /* Create our drivetrain in here */
        public static TalonSRX _rightTalon = new TalonSRX(1);
        public static VictorSPX _leftVictor = new VictorSPX(2);

        /* Gamepad */
        public static GameController _gamepad = new GameController(CTRE.Phoenix.UsbHostDevice.GetInstance(1), 0);
    }
}
