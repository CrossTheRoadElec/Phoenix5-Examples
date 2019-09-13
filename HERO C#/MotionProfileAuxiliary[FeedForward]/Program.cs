using System;
using System.Threading;
using Microsoft.SPOT;
using MotionProfileAuxiliary.Platform;
using Microsoft.SPOT.Hardware;
using CTRE.Phoenix.MotorControl;
using CTRE.Phoenix.Motion;
using System.Text;
using CTRE.Phoenix.Sensors;

namespace MotionProfileAuxiliary
{
    public class Program
    {
        static int _timeToPrint = 0;
        static int _timeToColumns = 0;
        static StringBuilder _sb = new StringBuilder();
        static SetValueMotionProfile _motionProfileSet = SetValueMotionProfile.Disable;
        static MotionProfileStatus _motionProfileStatus = new MotionProfileStatus();

        public static  void Main()
        {
			/* Factory Default all hardware to prevent unexpected behaviour */
			Hardware._rightTalon.ConfigFactoryDefault();
			Hardware._leftTalon.ConfigFactoryDefault();
			Hardware._leftVictor.ConfigFactoryDefault();
			Hardware._pidgey.ConfigFactoryDefault();
			/* Set neutral mode */
			Hardware._rightTalon.SetNeutralMode(NeutralMode.Brake);
            Hardware._leftVictor.SetNeutralMode(NeutralMode.Brake);
            Hardware._leftTalon.SetNeutralMode(NeutralMode.Brake);

            /** Feedback Sensor Configuration [Remote Sum] */

            /* Configure the left Talon's selected sensor as local QuadEncoder */
            Hardware._leftTalon.ConfigSelectedFeedbackSensor(   FeedbackDevice.QuadEncoder,     // Local Feedback Source
                                                                Constants.PID_PRIMARY,          // PID Slot for Source [0, 1]
                                                                Constants.kTimeoutMs);          // Configuration Timeout

            /* Configure the Remote Talon's selected sensor as a remote sensor for the right Talon */
            Hardware._rightTalon.ConfigRemoteFeedbackFilter(Hardware._leftTalon.GetDeviceID(),                              // Device ID of Source
                                                            RemoteSensorSource.RemoteSensorSource_TalonSRX_SelectedSensor,  // Remote Feedback Source
                                                            Constants.REMOTE_0,                                             // Source number [0, 1]
                                                            Constants.kTimeoutMs);                                          // Configuration Timeout

            /* Setup Sum signal to be used for Distance */
            Hardware._rightTalon.ConfigSensorTerm(SensorTerm.SensorTerm_Sum0, FeedbackDevice.RemoteSensor0, Constants.kTimeoutMs); // Feedback Device of Remote Talon
            Hardware._rightTalon.ConfigSensorTerm(SensorTerm.SensorTerm_Sum1, FeedbackDevice.QuadEncoder, Constants.kTimeoutMs);   // Quadrature Encoder of current Talon

            /* Configure Sum [Sum of both QuadEncoders] to be used for Primary PID Index */
            Hardware._rightTalon.ConfigSelectedFeedbackSensor(FeedbackDevice.SensorSum, Constants.PID_PRIMARY, Constants.kTimeoutMs);

            /* Scale Feedback by 0.5 to half the sum of Distance */
            Hardware._rightTalon.ConfigSelectedFeedbackCoefficient( 0.5f,                   // Coefficient
                                                                    Constants.PID_PRIMARY,  // PID Slot of Source 
                                                                    Constants.kTimeoutMs);  // Configuration Timeout
            
            /* Configure output and sensor direction */
            Hardware._rightTalon.SetInverted(true);
            Hardware._leftVictor.SetInverted(false);    // Output on victor
            Hardware._rightTalon.SetSensorPhase(true);
            Hardware._leftTalon.SetSensorPhase(true);   // Talon only used for QuadEncoder sensor

            /* Set status frame periods to ensure we don't have stale data */
            Hardware._rightTalon.SetStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 20, Constants.kTimeoutMs);
            Hardware._rightTalon.SetStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 20, Constants.kTimeoutMs);
            Hardware._rightTalon.SetStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 20, Constants.kTimeoutMs);
            Hardware._leftTalon.SetStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10, Constants.kTimeoutMs);

            /* Configure neutral deadband */
            Hardware._rightTalon.ConfigNeutralDeadband(Constants.kNeutralDeadband, Constants.kTimeoutMs);
            Hardware._leftVictor.ConfigNeutralDeadband(Constants.kNeutralDeadband, Constants.kTimeoutMs);

            /* Motion Magic/Profile Configurations */
            Hardware._rightTalon.ConfigMotionAcceleration(2000, Constants.kTimeoutMs);
            Hardware._rightTalon.ConfigMotionCruiseVelocity(2000, Constants.kTimeoutMs);

            /* Configure max peak output [Open and closed loop modes]
             * Can use configClosedLoopPeakOutput() for only closed Loop modes
             */
            Hardware._rightTalon.ConfigPeakOutputForward(+1.0f, Constants.kTimeoutMs);
            Hardware._rightTalon.ConfigPeakOutputReverse(-1.0f, Constants.kTimeoutMs);
            Hardware._leftVictor.ConfigPeakOutputForward(+1.0f, Constants.kTimeoutMs);
            Hardware._leftVictor.ConfigPeakOutputReverse(-1.0f, Constants.kTimeoutMs);

            /* FPID Gains for Motion Profile */
            Hardware._rightTalon.Config_kP(Constants.kSlot_MotProf, Constants.kGains_MotProf.kP, Constants.kTimeoutMs);
            Hardware._rightTalon.Config_kI(Constants.kSlot_MotProf, Constants.kGains_MotProf.kI, Constants.kTimeoutMs);
            Hardware._rightTalon.Config_kD(Constants.kSlot_MotProf, Constants.kGains_MotProf.kD, Constants.kTimeoutMs);
            Hardware._rightTalon.Config_kF(Constants.kSlot_MotProf, Constants.kGains_MotProf.kF, Constants.kTimeoutMs);
            Hardware._rightTalon.Config_IntegralZone(Constants.kSlot_MotProf, (int)Constants.kGains_MotProf.kIzone, Constants.kTimeoutMs);
            Hardware._rightTalon.ConfigClosedLoopPeakOutput(Constants.kSlot_MotProf, Constants.kGains_MotProf.kPeakOutput, Constants.kTimeoutMs);
            Hardware._rightTalon.ConfigAllowableClosedloopError(Constants.kSlot_MotProf, 0, Constants.kTimeoutMs);

            /* 1ms per loop.  PID loop can be slowed down if need be. */
            int closedLoopTimeMs = 1;
            Hardware._rightTalon.ConfigSetParameter(CTRE.Phoenix.LowLevel.ParamEnum.ePIDLoopPeriod, closedLoopTimeMs, 0x00, 0, Constants.kTimeoutMs);   // Primary
            Hardware._rightTalon.ConfigSetParameter(CTRE.Phoenix.LowLevel.ParamEnum.ePIDLoopPeriod, closedLoopTimeMs, 0x00, 1, Constants.kTimeoutMs);   // Turn (Auxiliary)

            /* False means talon's local output is PID0 + PID1, and other side Talon is PID0 - PID1
             * True means talon's local output is PID0 - PID1, and other side Talon is PID0 + PID1
             */
            Hardware._rightTalon.ConfigAuxPIDPolarity(false, Constants.kTimeoutMs);

            /* Increase the rate at which control frame is sent/recieved */
            Hardware._rightTalon.ChangeMotionControlFramePeriod(5);

            /* Configure base duration time of all trajectory points, which is then added to each points time duration */
            Hardware._rightTalon.ConfigMotionProfileTrajectoryPeriod(0, Constants.kTimeoutMs);

            /* Latched values to detect on-press events for buttons */
            bool[] _btns = new bool[Constants.kNumButtonsPlusOne];
            bool[] btns = new bool[Constants.kNumButtonsPlusOne];

            /* Initialize */
            bool _state = false;
            bool _firstCall = true;
            bool _direction = false;
            bool _MPComplete = false;
            ZeroSensors();

            while (true)
            {
                /* Enable motor controller output if gamepad is connected */
                if (Hardware._gamepad.GetConnectionStatus() == CTRE.Phoenix.UsbDeviceConnection.Connected)
                    CTRE.Phoenix.Watchdog.Feed();

                /* Joystick processing */
                float leftY = -1 * Hardware._gamepad.GetAxis(1);
                float rightX = +1 * Hardware._gamepad.GetAxis(2);
                CTRE.Phoenix.Util.Deadband(ref leftY);
                CTRE.Phoenix.Util.Deadband(ref rightX);

                /* Button processing */
                Hardware._gamepad.GetButtons( btns);
                if (btns[1] && !_btns[1])
                {
                    /* Enter/Exit reverse direction Motion Profile */
                    _state = !_state;
                    _firstCall = true;
                    _direction = false;     // Go reverse if X-Button pressed
                }else if(btns[3] && !_btns[3])
                {
                    /* Enter/Exit forward direction Motion Profile */
                    _state = !_state;
                    _firstCall = true;
                    _direction = true;      // Go forward if B-Button pressed
                }else if (btns[2] && !_btns[2])
                {
                    /* Zero all sensors used in example, only manual when in ArcadeDrive state */
                    if(_state == false)
                        ZeroSensors();
                }
                System.Array.Copy(btns, _btns, Constants.kNumButtonsPlusOne);

                /* Push/Clear Trajectory points */
                Hardware._rightTalon.ProcessMotionProfileBuffer();
                Thread.Sleep(5);

                /* Update motion profile status every loop */
                Hardware._rightTalon.GetMotionProfileStatus(_motionProfileStatus);

                if (!_state)
                {
                    if (_firstCall)
                        Debug.Print(("This is basic Arcade Drive with Arbitrary Feed-forward.\n") + 
                                    ("Enter/Exit Motion Profile Mode using Button 1 or 3. (X-Button or B-Button)\n") +
                                    ("Button 1 is reverse direction and Button 3 is forward direction. FeedForward applied from left thumbstick.\n"));
                    
                    /* Use Arbitrary FeedForward to create an Arcade Drive Control by modifying the forward output */
                    Hardware._rightTalon.Set(ControlMode.PercentOutput, leftY, DemandType.ArbitraryFeedForward, -rightX);
                    Hardware._leftVictor.Set(ControlMode.PercentOutput, leftY, DemandType.ArbitraryFeedForward, +rightX);
                }
                else
                {
                    if (_firstCall)
                    {
                        Debug.Print("Motion Profile will start once all trajectory points have been pushed into the buffer.\n" +
                                    "Custom FeedForward can be applied by the left thumb stick's Y-axis.\n");
                        ZeroSensors();

                        /* Disable Motion Profile to clear IsLast */
                        _motionProfileSet = SetValueMotionProfile.Disable;
                        Hardware._rightTalon.Set(ControlMode.MotionProfile, (int)_motionProfileSet);
                        Thread.Sleep(10);

                        /* Reset trajectory points*/
                        Hardware._rightTalon.ClearMotionProfileHasUnderrun();
                        Hardware._rightTalon.ClearMotionProfileTrajectories();
                        TrajectoryPoint point = new TrajectoryPoint();

                        /* Fill trajectory points */
                        for (uint i = 0; i < MotionProfile.kNumPoints; ++i)
                        {
                            /* Determine direction */
                            float direction = _direction ? +1 : -1;

                            /* Calculate Point's position, velocity, and heading */
                            point.position = direction * (float)MotionProfile.Points[i][0] * Constants.kSensorUnitsPerRotation;         // Convert from rotations to sensor units
                            point.velocity = direction * (float)MotionProfile.Points[i][1] * Constants.kSensorUnitsPerRotation / 600;   // Convert from RPM to sensor units per 100 ms.
                            point.headingDeg = 0;                                                                                       // Not used in this example

                            /* Define whether a point is first or last in trajectory buffer */
                            point.isLastPoint = (i + 1 == MotionProfile.kNumPoints) ? true : false;
                            point.zeroPos = (i == 0) ? true : false;

                            /* Slot Index provided through trajectory points rather than SelectProfileSlot() */
                            point.profileSlotSelect0 = Constants.kSlot_MotProf;

                            /* All points have the same duration of 10ms in this example */
                            point.timeDur = TrajectoryPoint.TrajectoryDuration.TrajectoryDuration_10ms;

                            /* Push point into buffer that will be proccessed with ProcessMotionProfileBuffer() */
                            Hardware._rightTalon.PushMotionProfileTrajectory(point);
                        }

                        /* Send a few points for initialization */
                        for (int i = 0; i < 5; ++i)
                            Hardware._rightTalon.ProcessMotionProfileBuffer();

                        _motionProfileSet = SetValueMotionProfile.Enable;

                        _MPComplete = false;
                    }

                    /* Telemetry */
                    if (_motionProfileStatus.activePointValid && _motionProfileStatus.isLast && _MPComplete == false)
                    {
                        Debug.Print("Motion Profile complete, holding final trajectory point.\n");
                        _MPComplete = true;
                    }

                    if (_MPComplete == false)
                        Instrument();

                    /* Calcluate FeedForward from gamepad */
                    float feedForward = leftY * 0.50f;
                    
                    /* Configured for Motion Profile on Quad Encoders' Sum and Arbitrary FeedForward on rightY */
                    Hardware._rightTalon.Set(ControlMode.MotionProfile, (int)_motionProfileSet, DemandType.ArbitraryFeedForward, feedForward);
                    Hardware._leftVictor.Follow(Hardware._rightTalon);
                }
                _firstCall = false;

                Thread.Sleep(5);
            }
        }

        /** Zero all sensors used in Auxiliary Example */
        public static void ZeroSensors()
        {
            Hardware._rightTalon.GetSensorCollection().SetQuadraturePosition(0, Constants.kTimeoutMs);
            Hardware._leftTalon.GetSensorCollection().SetQuadraturePosition(0, Constants.kTimeoutMs);
            Hardware._pidgey.SetYaw(0, Constants.kTimeoutMs);
            Hardware._pidgey.SetAccumZAngle(0, Constants.kTimeoutMs);
            Debug.Print("[Sensors] All sensors are zeroed.\n");
        }

        /** Formats various values and siginal into a readable format...
         * topCnt btmCnt  setval HasUndr IsUnder IsVal   IsLast VelOnly  TargetPos[AndVelocity] Pos[AndVelocity]
         */
        public static void Instrument()
        {
            if (--_timeToColumns <= 0)
            {
                _timeToColumns = 100;
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
                _timeToPrint = 10;

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
                _sb.Append(_motionProfileStatus.isLast ? "   1   \t" : "       \t");

                _sb.Append(Hardware._rightTalon.GetActiveTrajectoryPosition());
                _sb.Append("[");
                _sb.Append(Hardware._rightTalon.GetActiveTrajectoryVelocity());
                _sb.Append("]\t");

                _sb.Append("\t\t\t");
                _sb.Append(Hardware._rightTalon.GetSelectedSensorPosition(0));
                _sb.Append("[");
                _sb.Append(Hardware._rightTalon.GetSelectedSensorVelocity(0));
                _sb.Append("]");

                Debug.Print(_sb.ToString());
            }
        }
    }
}
