using System;
using System.Threading;
using Microsoft.SPOT;
using MotionProfileArcFeedForward.Platform;
using Microsoft.SPOT.Hardware;
using CTRE.Phoenix.MotorControl;
using CTRE.Phoenix.Motion;
using CTRE.Phoenix.Sensors;

namespace MotionProfileArcFeedForward
{
    public class Program
    {
        static SetValueMotionProfile _motionProfileSet = SetValueMotionProfile.Disable;
        static MotionProfileStatus _motionProfileStatus = new MotionProfileStatus();

        public static void Main()
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

            /** Feedback Sensor Configuration [Remote Sum and Yaw] */

            /* Configure the left Talon's selected sensor as local QuadEncoder */
            Hardware._leftTalon.ConfigSelectedFeedbackSensor(   FeedbackDevice.QuadEncoder,     // Local Feedback Source
                                                                Constants.PID_PRIMARY,          // PID Slot for Source [0, 1]
                                                                Constants.kTimeoutMs);          // Configuration Timeout

            /* Configure the Remote Talon's selected sensor as a remote sensor for the right Talon */
            Hardware._rightTalon.ConfigRemoteFeedbackFilter(Hardware._leftTalon.GetDeviceID(),                              // Device ID of Source
                                                            RemoteSensorSource.RemoteSensorSource_TalonSRX_SelectedSensor,  // Remote Feedback Source
                                                            Constants.REMOTE_0,                                             // Source number [0, 1]
                                                            Constants.kTimeoutMs);                                          // Configuration Timeout

            /* Configure the Pigeon IMU to the other Remote Slot on the Right Talon */
            Hardware._rightTalon.ConfigRemoteFeedbackFilter(Hardware._pidgey.GetDeviceID(),
                                                            RemoteSensorSource.RemoteSensorSource_Pigeon_Yaw,
                                                            Constants.REMOTE_1,
                                                            Constants.kTimeoutMs);

            /* Setup Sum signal to be used for Distance */
            Hardware._rightTalon.ConfigSensorTerm(SensorTerm.SensorTerm_Sum0, FeedbackDevice.RemoteSensor0, Constants.kTimeoutMs); // Feedback Device of Remote Talon
            Hardware._rightTalon.ConfigSensorTerm(SensorTerm.SensorTerm_Sum1, FeedbackDevice.QuadEncoder, Constants.kTimeoutMs);   // Quadrature Encoder of current Talon

            /* Configure Sum [Sum of both QuadEncoders] to be used for Primary PID Index */
            Hardware._rightTalon.ConfigSelectedFeedbackSensor(FeedbackDevice.SensorSum, Constants.PID_PRIMARY, Constants.kTimeoutMs);

            /* Scale Feedback by 0.5 to half the sum of Distance */
            Hardware._rightTalon.ConfigSelectedFeedbackCoefficient( 0.5f,                   // Coefficient
                                                                    Constants.PID_PRIMARY,  // PID Slot of Source 
                                                                    Constants.kTimeoutMs);  // Configuration Timeout

            /* Configure Remote Slot 1 [Pigeon IMU's Yaw] to be used for Auxiliary PID Index */
            Hardware._rightTalon.ConfigSelectedFeedbackSensor(FeedbackDevice.RemoteSensor1, Constants.PID_TURN, Constants.kTimeoutMs);

            /* Scale the Feedback Sensor using a coefficient (Configured for 3600 units of resolution) */
            Hardware._rightTalon.ConfigSelectedFeedbackCoefficient(Constants.kTurnTravelUnitsPerRotation / Constants.kPigeonUnitsPerRotation,
                                                                    Constants.PID_TURN,
                                                                    Constants.kTimeoutMs);

            /* Configure output and sensor direction */
            Hardware._rightTalon.SetInverted(true);
            Hardware._leftVictor.SetInverted(false);    // Output on victor
            Hardware._rightTalon.SetSensorPhase(true);
            Hardware._leftTalon.SetSensorPhase(true);   // Talon used for QuadEncoder sensor

            /* Set status frame periods to ensure we don't have stale data */
            Hardware._rightTalon.SetStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 20, Constants.kTimeoutMs);
            Hardware._rightTalon.SetStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 20, Constants.kTimeoutMs);
            Hardware._rightTalon.SetStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 20, Constants.kTimeoutMs);
            Hardware._rightTalon.SetStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 20, Constants.kTimeoutMs);
            Hardware._pidgey.SetStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR, 10, Constants.kTimeoutMs);
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

            /* FPID Gains for Arc of Motion Profile */
            Hardware._rightTalon.Config_kP(Constants.kSlot_Turning, Constants.kGains_Turning.kP, Constants.kTimeoutMs);
            Hardware._rightTalon.Config_kI(Constants.kSlot_Turning, Constants.kGains_Turning.kI, Constants.kTimeoutMs);
            Hardware._rightTalon.Config_kD(Constants.kSlot_Turning, Constants.kGains_Turning.kD, Constants.kTimeoutMs);
            Hardware._rightTalon.Config_kF(Constants.kSlot_Turning, Constants.kGains_Turning.kF, Constants.kTimeoutMs);
            Hardware._rightTalon.Config_IntegralZone(Constants.kSlot_Turning, (int)Constants.kGains_Turning.kIzone, Constants.kTimeoutMs);
            Hardware._rightTalon.ConfigClosedLoopPeakOutput(Constants.kSlot_Turning, Constants.kGains_Turning.kPeakOutput, Constants.kTimeoutMs);
            Hardware._rightTalon.ConfigAllowableClosedloopError(Constants.kSlot_Turning, 0, Constants.kTimeoutMs);

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
            int _state = 0;
            bool _firstCall = true;
            bool _direction = false;
            bool _MPComplete = false;
            double _targetHeading = 0;

            ZeroSensors();

            while (true)
            {
                /* Enable motor controller output if gamepad is connected */
                if (Hardware._gamepad.GetConnectionStatus() == CTRE.Phoenix.UsbDeviceConnection.Connected)
                    CTRE.Phoenix.Watchdog.Feed();

                /* Joystick processing */
                float leftY = -1 * Hardware._gamepad.GetAxis(1);
                float rightX = +1 * Hardware._gamepad.GetAxis(2);
                float leftX = -1 * Hardware._gamepad.GetAxis(5);
                CTRE.Phoenix.Util.Deadband(ref leftY);
                CTRE.Phoenix.Util.Deadband(ref rightX);
                CTRE.Phoenix.Util.Deadband(ref leftX);

                /* Button processing */
                Hardware._gamepad.GetButtons( btns);
                if (btns[1] && !_btns[1])
                {
                    /* Enter/Exit reverse direction Motion Profile */
                    _firstCall = true;
                    if (_state == 0)
                    {
                        _state = 1;
                        _direction = false;
                    }
                    else
                        _state = 0;
                }else if (btns[3] && !_btns[3])
                {
                    /* Enter/Exit forward direction Motion Profile */
                    _firstCall = true;
                    if (_state == 0) {
                        _state = 1;
                        _direction = true;
                    }
                    else
                        _state = 0;
                }else if (btns[2] && !_btns[2])
                {
                    /* Progress to next state and latch target heading */
                    if (_state == 1)
                    {
                        _firstCall = true;
                        _state = 2;
                    }
                }else if (btns[4] && !_btns[4])
                {
                    /* Only allow the ability to zero sensors when not performing Motion Profile (safety) */
                    if (_state == 0 || _state == 1)
                        ZeroSensors();
                }
                System.Array.Copy(btns, _btns, Constants.kNumButtonsPlusOne);

                /* Push/Clear Trajectory points */
                Hardware._rightTalon.ProcessMotionProfileBuffer();
                Thread.Sleep(5);

                /* Update motion profile status every loop */
                Hardware._rightTalon.GetMotionProfileStatus(_motionProfileStatus);

                if (_state == 0)
                {
                    if (_firstCall)
                        Debug.Print("This is basic Arcade Drive with Arbitrary Feed-forward.\n" +
                                    "Enter/Exit Motion Profile Arc using Button 1 or 3. (X-Button or B-Button)\n" +
                                    "Button 1 is reverse direction and Button 3 is forward direction. Feedforward applied from left thumbstick.\n");

                    /* Use Arbitrary FeedForward to create an Arcade Drive Control by modifying the forward output */
                    Hardware._rightTalon.Set(ControlMode.PercentOutput, leftY, DemandType.ArbitraryFeedForward, -rightX);
                    Hardware._leftVictor.Set(ControlMode.PercentOutput, leftY, DemandType.ArbitraryFeedForward, +rightX);

                }else if (_state == 1)
                {
                    if(_firstCall)
                        Debug.Print("You have entered Motion Profile Arc, now select the desired final heading.\n" +
                                    "Select your targetheading by using the left stick X-axis [3600, -3600] and press Button 2 to start.\n" +
                                    "Press either Button 1 or 3 to return back to ArcadeDriveAuxilary.\n");

                    /* Update target heading with joystick, which will be latched once user requests next state */
                    _targetHeading = Constants.kTurnTravelUnitsPerRotation * leftX * -1;
                    Debug.Print("Heading: " + _targetHeading);

                }else if(_state == 2)
                {
                    if (_firstCall)
                    {
                        Debug.Print("Motion Profile Arc will start once all trajectory points have been pushed into the buffer.\n" +
                                    "Custom FeedForward can be applied by the left thumb stick's Y-axis.\n");
                        ZeroPosition();

                        /* Disable Motion Profile to clear IsLast */
                        _motionProfileSet = SetValueMotionProfile.Disable;
                        Hardware._rightTalon.Set(ControlMode.MotionProfile, (int)_motionProfileSet);
                        Thread.Sleep(10);

                        /* Reset trajectory points*/
                        Hardware._rightTalon.ClearMotionProfileHasUnderrun();
                        Hardware._rightTalon.ClearMotionProfileTrajectories();
                        TrajectoryPoint point = new TrajectoryPoint();

                        /* Pull the final target distance, we will use this for heading generation */
                        double finalPositionRot = MotionProfile.Points[MotionProfile.kNumPoints - 1][0];

                        /* Fill trajectory points */
                        for (uint i = 0; i < MotionProfile.kNumPoints; ++i)
                        {
                            /* Calculations */
                            double direction = _direction ? +1 : -1;
                            double positionRot = MotionProfile.Points[i][0];
                            double velocityRPM = MotionProfile.Points[i][1];
                            double heading = _targetHeading * positionRot / finalPositionRot;   // Scale heading progress to position progress

                            /* Point's Position, Velocity, and Heading */
                            point.position = direction * positionRot * Constants.kSensorUnitsPerRotation;           // Convert from rotations to sensor units
                            point.velocity = direction * velocityRPM * Constants.kSensorUnitsPerRotation / 600;     // Convert from RPM to sensor units per 100 ms.
                            point.headingDeg = heading;                                                             // Convert to degrees

                            /* Define whether a point is first or last in trajectory buffer */
                            point.isLastPoint = (i + 1 == MotionProfile.kNumPoints) ? true : false;
                            point.zeroPos = (i == 0) ? true : false;

                            /* Slot Index provided through trajectory points rather than SelectProfileSlot() */
                            point.profileSlotSelect0 = Constants.kSlot_MotProf;
                            point.profileSlotSelect1 = Constants.kSlot_Turning; 

                            /* All points have the same duration of 10ms in this example */
                            point.timeDur = TrajectoryPoint.TrajectoryDuration.TrajectoryDuration_10ms;

                            /* Push point into buffer that will be proccessed with ProcessMotionProfileBuffer() */
                            Hardware._rightTalon.PushMotionProfileTrajectory(point);
                        }

                        /* Send a few points for initialization */
                        for (int i = 0; i < 10; ++i)
                            Hardware._rightTalon.ProcessMotionProfileBuffer();

                        /* Enable Motion Profile */
                        _motionProfileSet = SetValueMotionProfile.Enable;

                        _MPComplete = false;
                    }

                    if (_motionProfileStatus.activePointValid && _motionProfileStatus.isLast && _MPComplete == false)
                    {
                        Debug.Print("Motion Profile complete, holding final trajectory point.\n");
                        _MPComplete = true;
                    }

                    /* Calculate FeedForward from gamepad */
                    float feedForward = leftY * 0.50f;

                    /* Push/Clear Trajectory points */
                    Hardware._rightTalon.ProcessMotionProfileBuffer();

                    /* Configured for Motion Profile Arc on Quad Encoders' Sum and Pigeon's Heading, while also providing an Arbitrary FeedForward on left X */
                    Hardware._rightTalon.Set(ControlMode.MotionProfileArc, (int)_motionProfileSet, DemandType.ArbitraryFeedForward, feedForward);
                    Hardware._leftVictor.Follow(Hardware._rightTalon, FollowerType.AuxOutput1);
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

        /** Zero all QuadEncoder Positions used in Auxiliary Example */
        static void ZeroPosition()
        {
            Hardware._rightTalon.GetSensorCollection().SetQuadraturePosition(0, Constants.kTimeoutMs);
            Hardware._leftTalon.GetSensorCollection().SetQuadraturePosition(0, Constants.kTimeoutMs);
            Debug.Print("[Position] All sensors are zeroed.\n");
        }
    }
}
