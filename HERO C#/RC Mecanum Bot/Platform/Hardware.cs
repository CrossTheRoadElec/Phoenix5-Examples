/**
 * Example Hardware list below.  
 * This robot platform includes CANifier, several Talons, and a Pigeon, and support for a Futaba 3Ch RC radio.
 */
using CTRE.Phoenix.MotorControl.CAN;
using CTRE.Phoenix.Drive;
using CTRE.Phoenix.Controller;
using CTRE.Phoenix;
using CTRE.Phoenix.Sensors;
namespace HERO_Mecanum_Drive_Example.Platform
{
    public static class Hardware
    {
        /* the hardware objects.  Use 'public static' because these are single objects. */

        public static TalonSRX leftFrnt = new TalonSRX(1);
        public static TalonSRX leftRear = new TalonSRX(2);
        public static TalonSRX rghtFrnt = new TalonSRX(4);
        public static TalonSRX rghtRear = new TalonSRX(3);
        public static Mecanum drivetrain = new Mecanum(leftFrnt, leftRear, rghtFrnt, rghtRear);

        public static GameController gamepad = new GameController(CTRE.Phoenix.UsbHostDevice.GetInstance(0), 0);

        public static CANifier canifier_LedStrip_RCRADIO = new CANifier(0);

        public static RCRadio3Ch Futaba3Ch = new RCRadio3Ch(canifier_LedStrip_RCRADIO);

        /** Pigeon is plugged into the left rear Talon via ribbon cable. */
        public static PigeonIMU Pigeon = new PigeonIMU(leftRear);

        public static CTRE.Phoenix.Motion.ServoParameters ParametersHoldHeading = new CTRE.Phoenix.Motion.ServoParameters();

        public static CTRE.Motion.ServoHoldHeadingWithImu ServoHoldHeading
            = new CTRE.Motion.ServoHoldHeadingWithImu(Hardware.Pigeon,
                                                        Hardware.drivetrain,
                                                        CTRE.Phoenix.Drive.Styles.BasicStyle.PercentOutput,
                                                        ParametersHoldHeading,
                                                        0,
                                                        0.2f);
    }
}
