/**
 * Task manageing the CANifier outputs to the LED strip.
 * 
 * Limit switches are automatic via Talon Tach and Talon SRXs.
 * @link http://www.ctr-electronics.com/talon-tach-tachometer-new-limit-switch.html
 */
using CTRE.Mechanical;
using CTRE.MotorControllers;
using Platform;

namespace Subsystem
{
    public class SubSystemArm
    {
        /* grab the gearbox reference for easy reach */
        SensoredGearbox _gearBox = Hardware.ArmGearBox;

        /* track which control mode we are in */
        ControlMode _controlMode = ControlMode.kPercentVbus;

        public SubSystemArm()
        {
            Setup();
        }

        public TalonSrx MotorController
        {
            get
            {
                return (TalonSrx)_gearBox.GetMaster();
            }
        }

        public void Setup()
        {
            TalonSrx armTalon = (TalonSrx)_gearBox.GetMaster();

            armTalon.SetStatusFrameRateMs(TalonSrx.StatusFrameRate.StatusFrameRateGeneral, 10); //Send updates every 10ms instead of 10ms
            armTalon.SetStatusFrameRateMs(TalonSrx.StatusFrameRate.StatusFrameRatePulseWidthMeas, 1); //1ms update insead of 100ms for encoder
            armTalon.SetSensorDirection(true); //reversed sensor
            armTalon.ConfigFwdLimitSwitchNormallyOpen(false);
            armTalon.ConfigRevLimitSwitchNormallyOpen(false);
            armTalon.EnableZeroSensorPositionOnReverseLimit(false); //enable on reverse limit
            armTalon.EnableZeroSensorPositionOnForwardLimit(false);
        }
        private void SetupPositionServo()
        {
            TalonSrx armTalon = (TalonSrx)_gearBox.GetMaster();

            armTalon.ConfigNominalOutputVoltage(0, 0);
            armTalon.ConfigPeakOutputVoltage(Constants.MAX_VOLTAGE, -Constants.MAX_VOLTAGE);
            armTalon.SetAllowableClosedLoopErr(0, Constants.TOLERANCE);

            armTalon.SetPID(0, Constants.KPARM, Constants.KIARM, Constants.KDARM);

            _controlMode = ControlMode.kPosition;

            armTalon.SelectProfileSlot(0);
            armTalon.SetMotionMagicAcceleration(60.0f);
            armTalon.SetMotionMagicCruiseVelocity(22.0f);

            armTalon.SetVoltageRampRate(0f);
        }

        private void SetupMotorOutput()
        {
            TalonSrx armTalon = (TalonSrx)_gearBox.GetMaster();

            //_gearBox.SetControlMode(ControlMode.kPercentVbus);
            _controlMode = ControlMode.kPercentVbus;

            armTalon.SetVoltageRampRate(16.0f);
        }

        public void SetTargetPos(float pos)
        {
            if (_controlMode != ControlMode.kPosition)
            {
                SetupPositionServo();
            }
            _gearBox.Set(pos, _controlMode);
        }

        public void SetPercentOutput(float percentOutput)
        {
            percentOutput = CTRE.Util.Cap(percentOutput, 0.20f);

            if (_controlMode != ControlMode.kPercentVbus)
            {
                SetupMotorOutput();
            }
            _gearBox.Set(percentOutput, _controlMode);
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
