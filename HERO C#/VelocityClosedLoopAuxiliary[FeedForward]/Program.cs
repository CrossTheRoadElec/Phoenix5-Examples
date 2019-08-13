using System;
using System.Threading;
using Microsoft.SPOT;
using VelociyClosedLoopAuxiliary.Platform;
using CTRE.Phoenix.MotorControl;
using CTRE.Phoenix.Sensors;

namespace VelociyClosedLoopAuxiliary
{
    public class Program
    {
        public static void Main()
        {
			/* Factory Default all hardware to prevent unexpected behaviour */
			Hardware._rightTalon.ConfigFactoryDefault();
			Hardware._leftTalon.ConfigFactoryDefault();
			Hardware._leftVictor.ConfigFactoryDefault();

			/* Disable drivetrain/motors */
			Hardware._rightTalon.Set(ControlMode.PercentOutput, 0);
            Hardware._leftVictor.Set(ControlMode.PercentOutput, 0);
            Hardware._leftTalon.Set(ControlMode.PercentOutput, 0);

            /* Set Neutral Mode */
            Hardware._rightTalon.SetNeutralMode(NeutralMode.Brake);
            Hardware._leftVictor.SetNeutralMode(NeutralMode.Brake);
            Hardware._leftTalon.SetNeutralMode(NeutralMode.Brake);

            /** Feedback Sensor Configuration [Remote Difference] */

            /* Configure the drivetrain's left side Feedback Sensor as a Quadrature Encoder */
            Hardware._leftTalon.ConfigSelectedFeedbackSensor(   FeedbackDevice.QuadEncoder,     // Local Feedback Source
                                                                Constants.PID_PRIMARY,          // PID Slot for Source [0, 1]
                                                                Constants.kTimeoutMs);          // Configuration Timeout

            /* Configure the left Talon's Selected Sensor to be a remote sensor for the right Talon */
            Hardware._rightTalon.ConfigRemoteFeedbackFilter(Hardware._leftTalon.GetDeviceID(),                              // Device ID of Source
                                                            RemoteSensorSource.RemoteSensorSource_TalonSRX_SelectedSensor,  // Remote Feedback Source
                                                            Constants.REMOTE_0,                                             // Source number [0, 1]
                                                            Constants.kTimeoutMs);                                          // Configuration Timeout

            /* Setup Sum signal to be used for Distance PID */
            Hardware._rightTalon.ConfigSensorTerm(SensorTerm.SensorTerm_Sum0, FeedbackDevice.RemoteSensor0, Constants.kTimeoutMs);    // Feedback Device of Remote Talon
            Hardware._rightTalon.ConfigSensorTerm(SensorTerm.SensorTerm_Sum1, FeedbackDevice.QuadEncoder, Constants.kTimeoutMs);      // Quadrature Encoder of current Talon

            /* Configure Sum [Sum of both QuadEncoders] to be used for Primary PID Index */
            Hardware._rightTalon.ConfigSelectedFeedbackSensor(FeedbackDevice.SensorSum, Constants.PID_PRIMARY, Constants.kTimeoutMs);

            /* Scale the Feedback Sensor using a coefficient */
            Hardware._rightTalon.ConfigSelectedFeedbackCoefficient( 0.5f,                       // Coefficient
                                                                    Constants.PID_PRIMARY,      // PID Slot of Source
                                                                    Constants.kTimeoutMs);		// Configuration Timeout

            /* Configure output and sensor direction */
            Hardware._rightTalon.SetInverted(true);
            Hardware._leftVictor.SetInverted(false);    // Output on victor
            Hardware._rightTalon.SetSensorPhase(true);
            Hardware._leftTalon.SetSensorPhase(true);   // Talon only used for sensor

            /* Set status frame periods to ensure we don't have stale data */
            Hardware._rightTalon.SetStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 10, Constants.kTimeoutMs);
            Hardware._rightTalon.SetStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 10, Constants.kTimeoutMs);
            Hardware._leftTalon.SetStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10, Constants.kTimeoutMs);

            /* Configure Neutral Deadband */
            Hardware._rightTalon.ConfigNeutralDeadband(Constants.kNeutralDeadband, Constants.kTimeoutMs);
            Hardware._leftVictor.ConfigNeutralDeadband(Constants.kNeutralDeadband, Constants.kTimeoutMs);

            /* Configure max peak output [Open and closed loop modes]
             * Can use configClosedLoopPeakOutput() for only closed Loop modes
             */
            Hardware._rightTalon.ConfigPeakOutputForward(+1.0f, Constants.kTimeoutMs);
            Hardware._rightTalon.ConfigPeakOutputReverse(-1.0f, Constants.kTimeoutMs);
            Hardware._leftVictor.ConfigPeakOutputForward(+1.0f, Constants.kTimeoutMs);
            Hardware._leftVictor.ConfigPeakOutputReverse(-1.0f, Constants.kTimeoutMs);

            /* FPID Gains for turn closed loop */
            Hardware._rightTalon.Config_kP(Constants.kSlot_Velocit, Constants.kGains_Velocit.kP, Constants.kTimeoutMs);
            Hardware._rightTalon.Config_kI(Constants.kSlot_Velocit, Constants.kGains_Velocit.kI, Constants.kTimeoutMs);
            Hardware._rightTalon.Config_kD(Constants.kSlot_Velocit, Constants.kGains_Velocit.kD, Constants.kTimeoutMs);
            Hardware._rightTalon.Config_kF(Constants.kSlot_Velocit, Constants.kGains_Velocit.kF, Constants.kTimeoutMs);
            Hardware._rightTalon.Config_IntegralZone(Constants.kSlot_Velocit, (int)Constants.kGains_Velocit.kIzone, Constants.kTimeoutMs);
            Hardware._rightTalon.ConfigClosedLoopPeakOutput(Constants.kSlot_Velocit, Constants.kGains_Velocit.kPeakOutput, Constants.kTimeoutMs);

            /* Allow an error of 0, always enforce closed loop */
            Hardware._rightTalon.ConfigAllowableClosedloopError(Constants.kSlot_Velocit, 0, Constants.kTimeoutMs);

            /* 1ms per loop.  PID loop can be slowed down if need be. */
            int closedLoopTimeMs = 1;
            Hardware._rightTalon.ConfigSetParameter(CTRE.Phoenix.LowLevel.ParamEnum.ePIDLoopPeriod, closedLoopTimeMs, 0x00, 0, Constants.kTimeoutMs);   // Primary
            Hardware._rightTalon.ConfigSetParameter(CTRE.Phoenix.LowLevel.ParamEnum.ePIDLoopPeriod, closedLoopTimeMs, 0x00, 1, Constants.kTimeoutMs);   // Turn (Auxiliary)

            /* False means talon's local output is PID0 + PID1, and other side Talon is PID0 - PID1
             * True means talon's local output is PID0 - PID1, and other side Talon is PID0 + PID1
             */
            Hardware._rightTalon.ConfigAuxPIDPolarity(false, Constants.kTimeoutMs);

            /* Latched values to detect on-press events for buttons */
            bool[] _btns = new bool[Constants.kNumButtonsPlusOne];
            bool[] btns = new bool[Constants.kNumButtonsPlusOne];

            /* Initialize */
            bool _state = false;
            bool _firstCall = true;

            ZeroSensors();

            while (true)
            {
                /* Enable motor controllers if gamepad connected */
                if (Hardware._gamepad.GetConnectionStatus() == CTRE.Phoenix.UsbDeviceConnection.Connected)
                    CTRE.Phoenix.Watchdog.Feed();

                /* Gamepad value processing */
                float forward = -1 * Hardware._gamepad.GetAxis(1);
                float feedForward = -1 * Hardware._gamepad.GetAxis(5);
                float turn = 1 * Hardware._gamepad.GetAxis(2);
                CTRE.Phoenix.Util.Deadband(ref forward);
                CTRE.Phoenix.Util.Deadband(ref turn);

                /* Button processing */
                Hardware._gamepad.GetButtons( btns);
                if (btns[2] && !_btns[2])
                {
                    _state = !_state;           // Toggle state
                    _firstCall = true;          // State change, do first call operation
                }
                else if (btns[1] && !_btns[1])
                {
                    ZeroSensors();              // Zero sensors
                }
                System.Array.Copy(btns, _btns, Constants.kNumButtonsPlusOne);

                if (!_state)
                {
                    if (_firstCall)
                        Debug.Print("This is basic Arcade Drive with Arbitrary Feed-forward.\n");

                    /* Use Arbitrary FeedForward to create an Arcade Drive Control by modifying the forward output */
                    Hardware._rightTalon.Set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, -turn);
                    Hardware._leftVictor.Set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, +turn);
                }
                else
                {
                    if (_firstCall)
                    {
                        Debug.Print("This is Velocity Closed Loop with a custom Feed Forward.");
                        Debug.Print("Travel [-500, 500] RPM while having the ability to add a FeedForward (right stick).\n ");
                        ZeroSensors();

                        /* Determine which slot affects which PID */
                        Hardware._rightTalon.SelectProfileSlot(Constants.kSlot_Distanc, Constants.PID_PRIMARY);
                    }

                    /* Calculate targets from gamepad inputs */
                    double target_RPM = forward * 500; /* +- 500 RPM */
                    double target_unitsPer100ms = target_RPM * Constants.kSensorUnitsPerRotation / 600.0;
                    double feedFwdTerm = feedForward * 0.25; /* how much to add to the close loop output */


                    /* Configured for Velocity Closed Loop on Quad Encoders' Sum and Arbitrary FeedForward on right stick */
                    Hardware._rightTalon.Set(ControlMode.Velocity, target_unitsPer100ms, DemandType.ArbitraryFeedForward, feedFwdTerm);
                    Hardware._leftVictor.Follow(Hardware._rightTalon);
                }
                _firstCall = false;

                Thread.Sleep(10);
            }
        }

        /** Zero all sensors used in Auxiliary Example */
        static void ZeroSensors()
        {
            Hardware._rightTalon.GetSensorCollection().SetQuadraturePosition(0, Constants.kTimeoutMs);
            Hardware._leftTalon.GetSensorCollection().SetQuadraturePosition(0, Constants.kTimeoutMs);
            Hardware._pidgey.SetYaw(0, Constants.kTimeoutMs);
            Hardware._pidgey.SetAccumZAngle(0, Constants.kTimeoutMs);
            Debug.Print("[Sensors] All sensors are zeroed.\n");
        }
    }
}
