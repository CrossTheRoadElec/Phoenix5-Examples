using CTRE;

namespace BalanceBot.Platform
{
    public static class Hardware
    {
        //Create our two motors
        public static CTRE.Phoenix.MotorControl.CAN.TalonSRX rightTalon = new CTRE.Phoenix.MotorControl.CAN.TalonSRX(0);
        public static CTRE.Phoenix.MotorControl.CAN.TalonSRX leftTalon = new CTRE.Phoenix.MotorControl.CAN.TalonSRX(1);
		public static CTRE.Phoenix.MotorControl.CAN.TalonSRX[] allTalons = {rightTalon, leftTalon};
        public static CTRE.Phoenix.Mechanical.SensoredGearbox rightGearbox = new CTRE.Phoenix.Mechanical.SensoredGearbox(4096, rightTalon, CTRE.Phoenix.MotorControl.FeedbackDevice.CTRE_MagEncoder_Relative);
        public static CTRE.Phoenix.Mechanical.SensoredGearbox leftGearbox = new CTRE.Phoenix.Mechanical.SensoredGearbox(4096, leftTalon, CTRE.Phoenix.MotorControl.FeedbackDevice.CTRE_MagEncoder_Relative);

        //Create our drivetrain
        public static CTRE.Phoenix.Drive.SensoredTank drivetrain = new CTRE.Phoenix.Drive.SensoredTank(leftGearbox, rightGearbox, true, false, 3.125f);

        //Create Pigeon
        public static CTRE.Phoenix.Sensors.PigeonIMU pidgey = new CTRE.Phoenix.Sensors.PigeonIMU(0);
        //Create Gamepad
        public static CTRE.Phoenix.Controller.GameController Gamepad = new CTRE.Phoenix.Controller.GameController(CTRE.Phoenix.UsbHostDevice.GetInstance(0), 0);

        //Create display module for debugging
        public static CTRE.Gadgeteer.Module.DisplayModule Display = new CTRE.Gadgeteer.Module.DisplayModule(CTRE.HERO.IO.Port1, CTRE.Gadgeteer.Module.DisplayModule.OrientationType.Landscape);
        public static Microsoft.SPOT.Font smallFont = Properties.Resources.GetFont(Properties.Resources.FontResources.small);
        public static Microsoft.SPOT.Font bigFont = Properties.Resources.GetFont(Properties.Resources.FontResources.NinaB);

        //Everything else
        public static Battery battery = new Platform.Battery(rightTalon);
    }
}
