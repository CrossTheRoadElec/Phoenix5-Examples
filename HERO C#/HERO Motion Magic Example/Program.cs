/**
 * Example using the Motion Magic Control Mode of Talon SRX and the Magnetic Encoder.  Other sensors can be used by
 * changing the selected sensor type below.

 * MotionMagic control mode requires Talon firmware 11.8 or greater.

 * The test setup is ...
 *      A single Talon SRX (Device ID 0) http://www.ctr-electronics.com/talon-srx.html
 *      A VEX VersaPlanetary Gearbox http://www.vexrobotics.com/versaplanetary.html 
 *      Gearbox uses the CTRE Magnetic Encoder http://www.vexrobotics.com/vexpro/all/new-for-2016/217-5046.html
 *      Ribbon cable http://www.ctr-electronics.com/talon-srx-data-cable-4-pack.html
 *
 *      Talon SRX ribbon cable is connected to the Magnetic Encoder.  This provies the Talon with rotar position.
 *      See Talon SRX Software Reference Manual for gain-tuning suggestions.
 *
 * Press the top left shoulder button for direct-control of Talon's motor output using the left-y-axis.
 * Press the bottom left shoulder button to set the target position of the Talon's closed loop servo 
 * using the left-y-axis.  Notice the geared output will ramp up initiallly then ramp down as it approaches 
 * the target position.
 */
using System.Threading;
using System;
using Microsoft.SPOT;

using CTRE.Phoenix;
using CTRE.Phoenix.Controller;
using CTRE.Phoenix.MotorControl;
using CTRE.Phoenix.MotorControl.CAN;

namespace HERO_Motion_Magic_Example
{
    public class Program
    {
        /** talon to control */
        private TalonSRX _talon = new TalonSRX(0);
        /** desired mode to put talon in */
        private ControlMode _mode = ControlMode.PercentOutput;
        /** attached gamepad to HERO, tested with Logitech F710 */
        private GameController _gamepad = new GameController(UsbHostDevice.GetInstance());
        /** constant slot to use */
        const int kSlotIdx = 0;
        /** How long to wait for receipt when setting a param.  Many setters take an optional timeout that API will wait for.
            This is benefical for initial setup (before movement), though typically not desired 
            when changing parameters concurrently with robot operation (gain scheduling for example).*/
        const int kTimeoutMs = 30;

        /**
         * Setup all of the configuration parameters.
         */
        public void SetupConfig()
        {
			/* Factory Default all hardware to prevent unexpected behaviour */
			_talon.ConfigFactoryDefault();
			/* specify sensor characteristics */
			_talon.ConfigSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0);
            _talon.SetSensorPhase(false); /* make sure positive motor output means sensor moves in position direction */
            

            /* brake or coast during neutral */
            _talon.SetNeutralMode(NeutralMode.Brake);

            /* closed-loop and motion-magic parameters */
            _talon.Config_kF(kSlotIdx, 0.1153f, kTimeoutMs); // 8874 native sensor units per 100ms at full motor output (+1023)
            _talon.Config_kP(kSlotIdx, 2.00f, kTimeoutMs);
            _talon.Config_kI(kSlotIdx, 0f, kTimeoutMs);
            _talon.Config_kD(kSlotIdx, 20f, kTimeoutMs);
            _talon.Config_IntegralZone(kSlotIdx, 0, kTimeoutMs);
            _talon.SelectProfileSlot(kSlotIdx, 0); /* select this slot */
            _talon.ConfigNominalOutputForward(0f, kTimeoutMs);
			_talon.ConfigNominalOutputReverse(0f, kTimeoutMs);
			_talon.ConfigPeakOutputForward(1.0f, kTimeoutMs);
			_talon.ConfigPeakOutputReverse(-1.0f, kTimeoutMs);
			_talon.ConfigMotionCruiseVelocity(8000, kTimeoutMs); // 8000 native units
            _talon.ConfigMotionAcceleration(16000, kTimeoutMs); // 16000 native units per sec, (0.5s to reach cruise velocity).

            /* Home the relative sensor, 
                alternatively you can throttle until limit switch,
                use an absolute signal like CtreMagEncoder_Absolute or analog sensor.
                */
            _talon.SetSelectedSensorPosition(0, kTimeoutMs);

        }
        /** spin in this routine forever */
        public void RunForever()
        {
            SetupConfig(); /* configuration */
            /* robot loop */
            while (true)
            {
                /* get joystick params */
                float leftY = -1f * _gamepad.GetAxis(1);
                bool btnTopLeftShoulder = _gamepad.GetButton(5);
                bool btnBtmLeftShoulder = _gamepad.GetButton(7);
                Deadband(ref leftY);

                /* keep robot enabled if gamepad is connected and in 'D' mode */
                if (_gamepad.GetConnectionStatus() == UsbDeviceConnection.Connected)
                    Watchdog.Feed();

                /* set the control mode based on button pressed */
                if (btnTopLeftShoulder)
                    _mode = ControlMode.PercentOutput;
                if (btnBtmLeftShoulder)
                    _mode = ControlMode.MotionMagic;

                /* calc the Talon output based on mode */
                if (_mode == ControlMode.PercentOutput)
                {
                    float output = leftY; // [-1, +1] percent duty cycle
                    _talon.Set(_mode, output);
                }
                else if (_mode == ControlMode.MotionMagic)
                {
                    float servoToRotation = leftY * 10;// [-10, +10] rotations
                    _talon.Set(_mode, servoToRotation);
                }
                /* instrumentation */
                Instrument.Process(_talon);

                /* wait a bit */
                System.Threading.Thread.Sleep(5);
            }
        }
        /** @param [in,out] value to zero if within plus/minus 10% */
        public static void Deadband(ref float val)
        {
            if (val > 0.10f) { /* do nothing */ }
            else if (val < -0.10f) { /* do nothing */ }
            else { val = 0; } /* clear val since its within deadband */
        }
        /** singleton instance and entry point into program */
        public static void Main()
        {
            Program program = new Program();
            program.RunForever();
        }
    }
}
