/**
 * Task manageing the CANifier outputs to the LED strip.
 */
using CTRE.Phoenix.Mechanical;
using CTRE.Motion;
using CTRE.Phoenix.MotorControl;
using CTRE.Phoenix.MotorControl.CAN;
using Platform;
using CTRE.Phoenix.LowLevel;

namespace Subsystem
{
    public class SubSystemWheel
    {
        /* grab the gearbox and talon object */
        Gearbox _gearBox = Hardware.WheelGearBox;
        TalonSRX _tal = Hardware.wheelTalon;

        float _targetSpeedRPM = 0;

        ServoParameters _ServoParameters = new ServoParameters();

        public SubSystemWheel()
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
			_tal.ConfigForwardLimitSwitchSource(CTRE.Phoenix.MotorControl.LimitSwitchSource.FeedbackConnector, CTRE.Phoenix.MotorControl.LimitSwitchNormal.NormallyOpen);
			_tal.ConfigReverseLimitSwitchSource(CTRE.Phoenix.MotorControl.LimitSwitchSource.FeedbackConnector, CTRE.Phoenix.MotorControl.LimitSwitchNormal.NormallyOpen);
			_tal.ConfigSelectedFeedbackSensor(CTRE.Phoenix.MotorControl.FeedbackDevice.CTRE_MagEncoder_Relative);
			_tal.SetStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 1);


        }

        public float MeasuredSpeed
        {
            get
            {
				/* sample the period*/
				int periodUs;
				 _tal.GetSensorCollection().GetPulseWidthRiseToRiseUs(out periodUs);
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
            _ServoParameters.P = 0.005f;
            _ServoParameters.I = 0 * 0.0001f;
			_ServoParameters.F =  6f / 2000f; // about 6V for 2000 RPM
            /* save the target */
            _targetSpeedRPM = speedRPM;
            /* get measured speed */
            float measuredSpeedRpm = MeasuredSpeed;
            /* robot controller level closed loop, replace with firmware close loop later */
            float output = _ServoParameters.PID(_targetSpeedRPM, measuredSpeedRpm, 0);
			_tal.Set(ControlMode.PercentOutput, (output/2));
        }

        public void SetPercentOutput(float percentOut)
        {
            _tal.Set(CTRE.Phoenix.MotorControl.ControlMode.PercentOutput,percentOut);
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
