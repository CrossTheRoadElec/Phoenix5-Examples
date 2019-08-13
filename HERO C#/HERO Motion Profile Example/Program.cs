/**
 * Example demonstrating the motion profile control mode of Talon SRX.
 * Press and release button5 (top left shoulder button on Logitech Gamepad) to stream a Motion Profile to Talon SRX and execute it.
 * Press and release button7 (bottom left shoulder button on Logitech Gamepad) to put Talon into PercentOutput mode, where left y axis stick 
 * will control the output.
 */
using System;
using System.Threading;
using Microsoft.SPOT;
using System.Text;

using CTRE.Phoenix;
using CTRE.Phoenix.Controller;
using CTRE.Phoenix.MotorControl;
using CTRE.Phoenix.MotorControl.CAN;
using CTRE.Phoenix.Motion;

namespace Hero_Motion_Profile_Example
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
        TalonSRX _talon = new TalonSRX(6);
        GameController _gamepad = new GameController(UsbHostDevice.GetInstance());
        bool[] _btns = new bool[10];
        bool[] _btnsLast = new bool[10];
        StringBuilder _sb = new StringBuilder();
        int _timeToPrint = 0;
        int _timeToColumns= 0;
        const int kTicksPerRotation = 4096;

        MotionProfileStatus _motionProfileStatus = new MotionProfileStatus();

        public void Run()
        {
			/* Factory Default all hardware to prevent unexpected behaviour */
			_talon.ConfigFactoryDefault();



			_talon.ConfigSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0);
            _talon.SetSensorPhase(false);

            _talon.Config_kP(0, 0.80f);
            _talon.Config_kI(0, 0f);
            _talon.Config_kD(0, 0f);
            _talon.Config_kF(0, 0.09724488664269079041176191004297f);
            _talon.SelectProfileSlot(0, 0);
            _talon.ConfigNominalOutputForward(0f, 50);
            _talon.ConfigNominalOutputReverse(0f, 50);
            _talon.ConfigPeakOutputForward(+1.0f, 50);
            _talon.ConfigPeakOutputReverse(-1.0f, 50);
			_talon.ChangeMotionControlFramePeriod(5);
            _talon.ConfigMotionProfileTrajectoryPeriod(0, 50);

            /* loop forever */
            while (true)
            {
                _talon.GetMotionProfileStatus(_motionProfileStatus);

                Drive();

                Watchdog.Feed();

                Instrument();

                Thread.Sleep(5);
            }
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
        void FillBtns(ref bool[] btns)
        {
            for (uint i = 1; i < btns.Length; ++i)
                btns[i] = _gamepad.GetButton(i);
        }
        void Drive()
        {
            FillBtns(ref _btns);
            float y = -1 * _gamepad.GetAxis(1);

            Deadband(ref y);

            _talon.ProcessMotionProfileBuffer();

            /* button handler, if btn5 pressed launch MP, if btn7 pressed, enter percent output mode */
            if (_btns[5] && !_btnsLast[5])
            {
                /* disable MP to clear IsLast */
                _talon.Set(ControlMode.MotionProfile, 0);
                Watchdog.Feed();
                Thread.Sleep(10);
                /* buffer new pts in HERO */
                TrajectoryPoint point = new TrajectoryPoint();
                _talon.ClearMotionProfileHasUnderrun();
                _talon.ClearMotionProfileTrajectories();
                for (uint i = 0; i < MotionProfile.kNumPoints; ++i)
                {
                    point.position = (float)MotionProfile.Points[i][0] * kTicksPerRotation; //convert  from rotations to sensor units
                    point.velocity = (float)MotionProfile.Points[i][1] * kTicksPerRotation / 600;  //convert from RPM to sensor units per 100 ms.
                    point.headingDeg = 0; //not used in this example
                    point.isLastPoint = (i + 1 == MotionProfile.kNumPoints) ? true : false;
                    point.zeroPos = (i == 0) ? true : false;
                    point.profileSlotSelect0 = 0;
                    point.profileSlotSelect1 = 0; //not used in this example
					point.timeDur = TrajectoryPoint.TrajectoryDuration.TrajectoryDuration_10ms;
                    _talon.PushMotionProfileTrajectory(point);
                }
                /* send the first few pts to Talon */
                for (int i = 0; i < 5; ++i)
                {
                    Watchdog.Feed();
                    Thread.Sleep(10);
                    _talon.ProcessMotionProfileBuffer();
                }
                /*start MP */
                _talon.Set(ControlMode.MotionProfile, 1);
            }
            else if (_btns[7] && !_btnsLast[7])
            {
                _talon.Set(ControlMode.PercentOutput, 0);
            }

            /* if not in motion profile mode, update output percent */
            if (_talon.GetControlMode() != ControlMode.MotionProfile)
            {
                _talon.Set(ControlMode.PercentOutput, y);
            }

            /* copy btns => btnsLast */
            System.Array.Copy(_btns, _btnsLast, _btns.Length);
        }
        void Instrument()
        {
            if (--_timeToColumns <= 0)
            {
                _timeToColumns = 400;
                _sb.Clear();
                _sb.Append("topCnt \t");
                _sb.Append("btmCnt \t");
                _sb.Append("setval \t");
                _sb.Append("HasUndr\t");
                _sb.Append("IsUnder\t");
                _sb.Append(" IsVal \t");
                _sb.Append(" IsLast\t");
                _sb.Append("VelOnly\t");
                _sb.Append(" TargetPos[AndVelocity] \t");
                _sb.Append("Pos[AndVelocity]");
                Debug.Print(_sb.ToString());
            }

            if (--_timeToPrint <= 0)
            {
                _timeToPrint = 40;

                _sb.Clear();
                _sb.Append(_motionProfileStatus.topBufferCnt);
                _sb.Append("\t\t");
                _sb.Append(_motionProfileStatus.btmBufferCnt);
                _sb.Append("\t\t");
                _sb.Append(_motionProfileStatus.outputEnable);
                _sb.Append("\t\t");
                _sb.Append(_motionProfileStatus.hasUnderrun ? "   1   \t" : "       \t");
                _sb.Append(_motionProfileStatus.isUnderrun ? "   1   \t" : "       \t");
                _sb.Append(_motionProfileStatus.activePointValid ? "   1   \t" : "       \t");
            
                _sb.Append(_motionProfileStatus.isLast  ? "   1   \t" : "       \t");

                _sb.Append(_talon.GetActiveTrajectoryPosition());
                _sb.Append("[");
                _sb.Append(_talon.GetActiveTrajectoryVelocity());
                _sb.Append("]\t");


                _sb.Append("\t\t\t");
                _sb.Append(_talon.GetSelectedSensorPosition(0));
                _sb.Append("[");
                _sb.Append(_talon.GetSelectedSensorVelocity(0));
                _sb.Append("]");

                Debug.Print(_sb.ToString());
            }
        }
    }
}