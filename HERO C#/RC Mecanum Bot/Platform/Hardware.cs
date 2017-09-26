/**
 * Example Hardware list below.  
 * This robot platform includes CANifier, several Talons, and a Pigeon, and support for a Futaba 3Ch RC radio.
 */
using CTRE.MotorControllers;

namespace HERO_Mecanum_Drive_Example.Platform
{
    public static class Hardware
    {
        /* the hardware objects.  Use 'public static' because these are single objects. */

        public static TalonSrx leftFrnt = new TalonSrx(1);
        public static TalonSrx leftRear = new TalonSrx(2);
        public static TalonSrx rghtFrnt = new TalonSrx(4);
        public static TalonSrx rghtRear = new TalonSrx(3);
        public static CTRE.Drive.Mecanum drivetrain = new CTRE.Drive.Mecanum(leftFrnt, leftRear, rghtFrnt, rghtRear);

        public static CTRE.Controller.GameController gamepad = new CTRE.Controller.GameController(CTRE.UsbHostDevice.GetInstance(0), 0);

        public static CTRE.CANifier canifier_LedStrip_RCRADIO = new CTRE.CANifier(0);

        public static CTRE.RCRadio3Ch Futaba3Ch = new CTRE.RCRadio3Ch(canifier_LedStrip_RCRADIO);

        /** Pigeon is plugged into the left rear Talon via ribbon cable. */
        public static CTRE.PigeonImu Pigeon = new CTRE.PigeonImu(leftRear);

        public static CTRE.ServoParameters ParametersHoldHeading = new CTRE.ServoParameters();

        public static CTRE.Motion.ServoHoldHeadingWithImu ServoHoldHeading
            = new CTRE.Motion.ServoHoldHeadingWithImu(Hardware.Pigeon,
                                                        Hardware.drivetrain,
                                                        CTRE.Drive.Styles.Basic.PercentOutput,
                                                        ParametersHoldHeading,
                                                        0,
                                                        0.2f);
    }
}
