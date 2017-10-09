/**
 * Task manageing the CANifier outputs to the LED strip.
 */
using CTRE.Mechanical;
using CTRE.Motion;
using CTRE.MotorControllers;
using Platform;

namespace Subsystem
{
    public class SubSystemWheel
    {
        /* grab the gearbox and talon object */
        Gearbox _gearBox = Hardware.WheelGearBox;
        TalonSrx _tal = Hardware.wheelTalon;

        float _targetSpeedRPM = 0;

        ServoParameters _ServoParameters = new ServoParameters();

        public SubSystemWheel()
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
            _tal.ConfigFwdLimitSwitchNormallyOpen(true);
            _tal.ConfigRevLimitSwitchNormallyOpen(true);
            _tal.SetControlMode(ControlMode.kVoltage); //voltage control mode
            _tal.SetFeedbackDevice(TalonSrx.FeedbackDevice.CtreMagEncoder_Relative); //sensor type
            _tal.SetStatusFrameRateMs(TalonSrx.StatusFrameRate.StatusFrameRatePulseWidthMeas, 1); //feedback to 1ms
        }

        public float MeasuredSpeed
        {
            get
            {
                /* sample the period*/
                uint periodUs = (uint)_tal.GetPulseWidthRiseToRiseUs();
                /* convert to frequency */
                float edgesPerMin;
                if (periodUs == 0)
                {
                    /* wheel is not spinning or sensor is disconnected */
                    edgesPerMin = 0;
                }
                else
                {
                    /* convert from us to EPM */
                    edgesPerMin = 60000000f / periodUs;
                }
                /* convert to RPM, assume 1 mark per rotation */
                float rpm = edgesPerMin / Constants.MarksPerRotation;

                return rpm;
            }
        }


        public void ServoToSpeed(float speedRPM)
        {
            /* close loop constants */
            _ServoParameters.P = 0.01f;
            _ServoParameters.I = 0 * 0.0001f;
            _ServoParameters.F = 6f / 2000f; // about 6V for 2000 RPM
            /* save the target */
            _targetSpeedRPM = speedRPM;
            /* get measured speed */
            float measuredSpeedRpm = MeasuredSpeed;
            /* robot controller level closed loop, replace with firmware close loop later */
            float output = _ServoParameters.PID(_targetSpeedRPM, measuredSpeedRpm, 0);
            _tal.Set(output);
        }

        public void SetPercentOutput(float percentOut)
        {
            _tal.Set(percentOut);
        }

        public void Stop()
        {
            SetPercentOutput(0);
        }
        public override string ToString()
        {
            return "SysWhl:RPM:" + MeasuredSpeed;
        }
    }
}
