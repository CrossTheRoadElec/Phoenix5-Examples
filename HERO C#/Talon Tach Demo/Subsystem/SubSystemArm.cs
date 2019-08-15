/**
 * Task manageing the CANifier outputs to the LED strip.
 * 
 * Limit switches are automatic via Talon Tach and Talon SRXs.
 * @link http://www.ctr-electronics.com/talon-tach-tachometer-new-limit-switch.html
 */
using CTRE.Phoenix.Mechanical;
using CTRE.Phoenix.MotorControl;
using CTRE.Phoenix.MotorControl.CAN;
using Platform;

namespace Subsystem
{
    public class SubSystemArm
    {
        /* grab the gearbox reference for easy reach */
        SensoredGearbox _gearBox = Hardware.ArmGearBox;

        /* track which control mode we are in */
        ControlMode _controlMode = ControlMode.PercentOutput;

        public SubSystemArm()
        {

            Setup();
        }

        public TalonSRX MotorController
        {
            get
            {
                return (TalonSRX)_gearBox.MasterMotorController;
            }
        }

        public void Setup()
        {
            TalonSRX armTalon = (TalonSRX)_gearBox.MasterMotorController;
			armTalon.SetInverted(false);
            armTalon.SetStatusFramePeriod(StatusFrame.Status_1_General_, 10); //Send updates every 10ms instead of 10ms
			armTalon.SetStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 1);
			armTalon.SetSensorPhase(true); //reversed sensor
			armTalon.ConfigForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
			armTalon.ConfigReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
			
			armTalon.ConfigClearPositionOnLimitR(false,10); //enable on reverse limit
			armTalon.ConfigClearPositionOnLimitF(false,10);
        }
        private void SetupPositionServo()
        {
            TalonSRX armTalon = (TalonSRX)_gearBox.MasterMotorController;

			armTalon.ConfigNominalOutputForward(0);
			armTalon.ConfigNominalOutputReverse(0);

			armTalon.ConfigPeakOutputForward(1);
			armTalon.ConfigPeakOutputReverse(-1);

			armTalon.ConfigAllowableClosedloopError(0,Constants.TOLERANCE);
			armTalon.Config_kP(Constants.KPARM);
			armTalon.Config_kI(Constants.KIARM);
			armTalon.Config_kD(Constants.KDARM);
            _controlMode = ControlMode.MotionMagic;

            armTalon.SelectProfileSlot(0);
            armTalon.ConfigMotionAcceleration(60);
            armTalon.ConfigMotionCruiseVelocity(22);

        }

        private void SetupMotorOutput()
        {
            TalonSRX armTalon = (TalonSRX)_gearBox.MasterMotorController;

            _controlMode = ControlMode.PercentOutput;

        }

        public void SetTargetPos(float pos)
        {
            if (_controlMode != ControlMode.MotionMagic)
            {
                SetupPositionServo();
            }
			_gearBox.Set(_controlMode, pos);
        }

        public void SetPercentOutput(float percentOutput)
        {
            percentOutput = CTRE.Phoenix.Util.Cap(percentOutput, 0.25f);

            if (_controlMode != ControlMode.PercentOutput)
            {
                SetupMotorOutput();
            }
			_gearBox.Set(_controlMode, percentOutput);

		}

		public void Stop()
        {
            SetPercentOutput(0);
        }
        public override string ToString()
        {
            return "SysArm:Pos:" + _gearBox.GetPosition() + "Vel:" + _gearBox.GetVelocity();
        }
    }
}
