/**
 * Example demonstrating the position closed-loop servo.
 * Tested with Logitech F350 USB Gamepad inserted into HERO.
 * 
 * Use the mini-USB cable to deploy/debug.
 *
 * Be sure to select the correct feedback sensor using SetFeedbackDevice() below.
 *
 * After deploying/debugging this to your HERO, first use the left Y-stick 
 * to throttle the Talon manually.  This will confirm your hardware setup.
 * Be sure to confirm that when the Talon is driving forward (green) the 
 * position sensor is moving in a positive direction.  If this is not the cause
 * flip the boolena input to the SetSensorDirection() call below.
 *
 * Once you've ensured your feedback device is in-phase with the motor,
 * use the button shortcuts to servo to target positions.  
 *
 * Tweak the PID gains accordingly.
 */
using System;
using System.Threading;
using Microsoft.SPOT;
using System.Text;

using CTRE.Phoenix;
using CTRE.Phoenix.Controller;
using CTRE.Phoenix.MotorControl;
using CTRE.Phoenix.MotorControl.CAN;

namespace Hero_Position_Servo_Example
{
    /** Simple stub to start our project */
    public class Program
    {
        static RobotApplication _robotApp = new RobotApplication();
        public static void Main()
        {
            while(true)
            {
                _robotApp.Run();
            }
        }
    }
    /**
     * The custom robot application.
     */
    public class RobotApplication
    {
        /** scalor to max throttle in manual mode. negative to make forward joystick positive */
        const float kJoystickScaler = -0.3f;

        /** hold bottom left shoulder button to enable motors */
        const uint kEnableButton = 7;

        /** make a talon with deviceId 0 */
        TalonSRX _talon = new TalonSRX(0);

        /** Use a USB gamepad plugged into the HERO */
        GameController _gamepad = new GameController(UsbHostDevice.GetInstance());

        /** hold the current button values from gamepad*/
        bool[] _btns = new bool[10];

        /** hold the last button values from gamepad, this makes detecting on-press events trivial */
        bool[] _btnsLast = new bool[10];

        /** some objects used for printing to the console */
        StringBuilder _sb = new StringBuilder();
        int _timeToPrint = 0;

        float _targetPosition = 0;

		/* nonzero to block the config until success, zero to skip checking */
		const int kTimeoutMs = 30;

		uint [] _debLeftY = { 0, 0 }; // _debLeftY[0] is how many times leftY is zero, _debLeftY[1] is how many times leftY is not zeero.

        public void Run()
        {
			/* Factory Default all hardware to prevent unexpected behaviour */
			_talon.ConfigFactoryDefault();


			/* first choose the sensor */
			_talon.ConfigSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, kTimeoutMs);
            _talon.SetSensorPhase(false);

            /* set closed loop gains in slot0 */
            _talon.Config_kP(0, 0.2f, kTimeoutMs); /* tweak this first, a little bit of overshoot is okay */
            _talon.Config_kI(0, 0f, kTimeoutMs); 
            _talon.Config_kD(0, 0f, kTimeoutMs);
            _talon.Config_kF(0, 0f, kTimeoutMs); /* For position servo kF is rarely used. Leave zero */

            /* use slot0 for closed-looping */
            _talon.SelectProfileSlot(0, 0);

            /* set the peak and nominal outputs, 1.0 means full */
            _talon.ConfigNominalOutputForward(0.0f, kTimeoutMs);
			_talon.ConfigNominalOutputReverse(0.0f, kTimeoutMs);
            _talon.ConfigPeakOutputForward(+1.0f, kTimeoutMs);
            _talon.ConfigPeakOutputReverse(-1.0f, kTimeoutMs);

			/* how much error is allowed?  This defaults to 0. */
			_talon.ConfigAllowableClosedloopError(0,0, kTimeoutMs);

			/* put in a ramp to prevent the user from flipping their mechanism in open loop mode */
			_talon.ConfigClosedloopRamp(0, kTimeoutMs);
			_talon.ConfigOpenloopRamp(1, kTimeoutMs);

            /* zero the sensor and throttle */
            ZeroSensorAndThrottle();

            /* loop forever */
            while (true)
            {
                Loop10Ms();

                //if (_gamepad.GetConnectionStatus() == CTRE.UsbDeviceConnection.Connected) // check if gamepad is plugged in OR....
                if(_gamepad.GetButton(kEnableButton)) // check if bottom left shoulder buttom is held down.
                {
                    /* then enable motor outputs*/
                    Watchdog.Feed();
                }

                /* print signals to Output window */
                Instrument();

                /* 10ms loop */
                Thread.Sleep(10);
            }
        }
        /**
         * Zero the sensor and zero the throttle.
         */
        void ZeroSensorAndThrottle()
        {
            _talon.SetSelectedSensorPosition(0, kTimeoutMs); /* start our position at zero, this example uses relative positions */
            _targetPosition = 0;
			/* zero throttle */
			_talon.Set(ControlMode.PercentOutput, 0);
            Thread.Sleep(100); /* wait a bit to make sure the Setposition() above takes effect before sampling */
        }
        void EnableClosedLoop()
        {
            /* user has let go of the stick, lets closed-loop whereever we happen to be */
            _talon.Set(ControlMode.Position, _targetPosition);
        }
        void Loop10Ms()
        {
            /* get all the buttons */
            FillBtns(ref _btns);

            /* get the left y stick, invert so forward is positive */
            float leftY = kJoystickScaler * _gamepad.GetAxis(1);
            Deadband(ref leftY);

            /* debounce the transition from nonzero => zero axis */
            float filteredY = leftY;

            if (filteredY != 0)
            {
                /* directly control the output */
                _talon.Set(ControlMode.PercentOutput, filteredY);
            }
            else if (_talon.GetControlMode() == ControlMode.PercentOutput)
            {
                _targetPosition = _talon.GetSelectedSensorPosition(0);

                /* user has let go of the stick, lets closed-loop whereever we happen to be */
                EnableClosedLoop();
            }

            /* if a button is pressed while stick is let go, servo position */
            if (filteredY == 0)
            {
                if (_btns[1])
                {
                    _targetPosition = _talon.GetSelectedSensorPosition(0); /* current position */
                    EnableClosedLoop();
                }
                else if(_btns[4])
                {
                    _targetPosition = +40960.0f; /* ten rotations forward */
                    EnableClosedLoop();
                }
                else if (_btns[2])
                {
                    _targetPosition = -40960.0f; /* ten rotations reverse */
                    EnableClosedLoop();
                }
            }

            /* copy btns => btnsLast */
            System.Array.Copy(_btns, _btnsLast, _btns.Length);
        }
        /**
         * @return a filter value for the y-axis.  Don't return zero unless we've been in deadband for a number of loops.
         *                                          This is only done because this example will throttle the motor with 
         */
        float FilterLeftY(float y, uint numLoop)
        {
            /* get the left y stick */
            float leftY = -1 * _gamepad.GetAxis(1);
            Deadband(ref leftY);
            if (leftY == 0)
            {
                _debLeftY[1] = 0;
                ++_debLeftY[0];
            }
            else
            {
                _debLeftY[0] = 0;
                ++_debLeftY[1];
            }

            if (_debLeftY[0] > numLoop)
                return 0;
            return y;
        }
        /**
         * If value is within 10% of center, clear it.
         */
        void Deadband(ref float value)
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
        /** throw all the gamepad buttons into an array */
        void FillBtns(ref bool[] btns)
        {
            for (uint i = 1; i < btns.Length; ++i)
                btns[i] = _gamepad.GetButton(i);
        }
        /** occasionally builds a line and prints to output window */
        void Instrument()
        {
            if (--_timeToPrint <= 0)
            {
                _timeToPrint = 20;
                _sb.Clear();
                _sb.Append( "pos=");
                _sb.Append(_talon.GetSelectedSensorPosition(0));
                _sb.Append(" vel=");
                _sb.Append(_talon.GetSelectedSensorVelocity(0));
                _sb.Append(" err=");
                _sb.Append(_talon.GetClosedLoopError(0));
                _sb.Append(" out%=");
                _sb.Append(_talon.GetMotorOutputPercent());
                Debug.Print(_sb.ToString());
            }
        }
    }
}
