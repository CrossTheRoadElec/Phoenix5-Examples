using System;
using DriveStraightVelocityAuxiliary.Platform;
using CTRE.Phoenix.MotorControl.CAN;
using CTRE.Phoenix.MotorControl;
using CTRE.Phoenix.Controller;
using CTRE.Phoenix.Sensors;

namespace DriveStraightVelocityAuxiliary.Platform
{
    public static class Constants
    {
        /**
         * How many joystick buttons to poll.
         * 10 means buttons[1,9] are polled, which is actually 9 buttons
         */
        public const int kNumButtonsPlusOne = 10;

        /**
         * How many sensor units per rotation.
         * Using CTRE Magnetic Encoder.
         * @link https://github.com/CrossTheRoadElec/Phoenix-Documentation#what-are-the-units-of-my-sensor
         */
        public const int kSensorUnitsPerRotation = 4096;

        /**
	     * Empirically measure what the difference between encoders per 360'
	     * Drive the robot in clockwise rotations and measure the units per rotation.
	     * Drive the robot in counter clockwise rotations and measure the units per rotation.
	     * Take the average of the two.
	     */
        public const int kEncoderUnitsPerRotation = 51711;


        /**
         * Using the config feature, scale units to 3600 per rotation.
         * This is nice as it keeps 0.1 deg resolution, and is fairly intuitive.
         */
        public const float kTurnTravelUnitsPerRotation = 3600;

        /**
         * set to zero to skip waiting for confirmation, set to nonzero to wait
         * and report to DS if action fails.
         */
        public const int kTimeoutMs = 30;

        /**
         * Motor neutral dead-band, set to the minimum 0.1%.
         */
        public const float kNeutralDeadband = 0.001f;

        /**
         * PID Gains may have to be adjusted based on the responsiveness of control loop
         *                                                                 kP    kI     kD    kF                 Iz   PeakOutput */
        public static Framework.Gains kGains_Distanc = new Framework.Gains(0.1f, 0.0f,  0.0f, 0.0f,              100f, 0.50f);
        public static Framework.Gains kGains_Turning = new Framework.Gains(2.0f, 0.0f,  4.0f, 0.0f,              200f, 1.00f);
        public static Framework.Gains kGains_Velocit = new Framework.Gains(0.1f, 0.0f, 20.0f, 1023.0f / 6800.0f, 300f, 0.50f); /* measured 6800 velocity units at full motor output */
        public static Framework.Gains kGains_MotProf = new Framework.Gains(1.0f, 0.0f,  0.0f, 1023.0f / 6800.0f, 400f, 1.00f); /* measured 6800 velocity units at    */

        /* We allow either a 0 or 1 when selecting an ordinal for remote devices [You can have up to 2 devices assigned remotely to a talon/victor] */
        public const int REMOTE_0 = 0;
        public const int REMOTE_1 = 1;
        /* We allow either a 0 or 1 when selecting a PID ID, where 0 is primary and 1 is auxiliary */
        public const int PID_PRIMARY = 0;
        public const int PID_TURN = 1;
        /* Firmware currently supports slots [0, 3] and can be used for either PID Set */
        public const int SLOT_0 = 0;
        public const int SLOT_1 = 1;
        public const int SLOT_2 = 2;
        public const int SLOT_3 = 3;
        /* ---- Named slots, used to clarify code ---- */
        public const int kSlot_Distanc = SLOT_0;
        public const int kSlot_Turning = SLOT_1;
        public const int kSlot_Velocit = SLOT_2;
        public const int kSlot_MotProf = SLOT_3;
    }

    public static class Hardware
    {
        /* MotorControllers and Sensors */
        public static TalonSRX _rightTalon = new TalonSRX(1);
        public static TalonSRX _leftTalon = new TalonSRX(2);
        public static VictorSPX _leftVictor = new VictorSPX(2);
        public static PigeonIMU _pidgey = new PigeonIMU(3);

        /* Gamepad */
        public static GameController _gamepad = new GameController(CTRE.Phoenix.UsbHostDevice.GetInstance(1), 0);
    }
}
