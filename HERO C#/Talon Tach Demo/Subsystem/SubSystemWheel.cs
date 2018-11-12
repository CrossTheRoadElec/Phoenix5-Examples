/**
 * Wheel Subsystem in TalonTach Example. 
 * Wheel initialized with Talon Tach Sensor and Voltage Compenstation
 * 
 * Provides methods for both Velocity Closed Loop and Percent Output
 */
using CTRE.Phoenix.Mechanical;
using CTRE.Phoenix.MotorControl;
using CTRE.Phoenix.MotorControl.CAN;
using Platform;
using Microsoft.SPOT;

namespace Subsystem
{
    public class SubSystemWheel
    {
		/* grab the gearbox and talon reference for easy reach */
		Gearbox _gearBox = Hardware.WheelGearBox;
        TalonSRX _tal = Hardware.wheelTalon;

		public TalonSRX MotorController
		{
			get
			{
				return Platform.Hardware.wheelTalon;
			}
		}

		/* General Setup */
		public void Initialize()
        {
			/* Configure Voltage compenstaion */
			_tal.ConfigVoltageCompSaturation(Constants.MAX_VOLTAGE);		// 12 Volts
			_tal.ConfigVoltageMeasurementFilter(32);						// Default 32 Samples per Avg
			_tal.EnableVoltageCompensation(true);							// Enable Voltage Compensation
		    
			/* Configure Selected Sensor */
            _tal.ConfigSelectedFeedbackSensor(FeedbackDevice.Tachometer, 0);				// Sensor type
			_tal.ConfigPulseWidthPeriod_EdgesPerRot(Constants.MarksPerRotation, 30);		// 6 EdgesPerRot + 30ms Timeout
			_tal.ConfigPulseWidthPeriod_FilterWindowSz(Constants.PWM_FilterWindowSize, 30);	// 1 SamplePerAvg + 30ms Timeout
		}

		/* Get Speed in RPM */
		public float MeasuredSpeed
        {
			get
			{
				/* GetSelectedSensorVelocity returns velocity in native units.
				 * Values used to convert native units to RPM can be found here...
				 * https://github.com/CrossTheRoadElec/Phoenix-Documentation#what-are-the-units-of-my-sensor
				 */
				int unitsPer100ms = _tal.GetSelectedSensorVelocity();
				int velocityRPM = unitsPer100ms * 600 / 1024;
				return velocityRPM;
			}
		}

		/* Perform Velocity Closed Loop drive */
        public void ServoToSpeed(float speedRPM)
        {
			/* PID Parameters for MotorController level Closed Loop */
			_tal.Config_kP(0.05f);
			_tal.Config_kI(0.000f);
			_tal.Config_kF(6f / 2000f);

			/* Calculate requested speed in RPM back into native units for Talon */
			double speedPer100ms = speedRPM / 600 * 4096;

			/* Set */
			_tal.Set(ControlMode.Velocity, speedPer100ms);
        }

		/* Perform Percent Output drive */
        public void SetPercentOutput(float percentOut)
        {
			/* Set */
            _tal.Set(ControlMode.PercentOutput, percentOut);
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
