/**
 * Arm Subsystem in TalonTach Example. 
 * Arm initialized with CTRE Mag Encoder and Hardware Limit Switches (TalonTachs)
 * Used to demonstrate the ability to use Talon Tach as Hardware LimitSwitches
 * 
 * Provides method Percent Output and initialization
 *  
 * TODO: Add proper Closed Loop Mode to Task Servo Arm
 * 
 * Limit switches are automatic via Talon Tach and Talon SRXs.
 * @link http://www.ctr-electronics.com/talon-tach-tachometer-new-limit-switch.html
 */

using Platform;
using CTRE.Phoenix.Mechanical;
using CTRE.Phoenix.MotorControl;
using CTRE.Phoenix.MotorControl.CAN;
using Microsoft.SPOT;

namespace Subsystem
{
    public class SubSystemArm
    {
        /* grab the gearbox and talon reference for easy reach */
        SensoredGearbox _gearBox = Hardware.ArmGearBox;
		TalonSRX _tal = Hardware.armTalon;

		/* track which control mode we are in */
		ControlMode _controlMode = ControlMode.PercentOutput;

        public TalonSRX MotorController
        {
            get
            {
				return (TalonSRX)_gearBox.MasterMotorController;
			}
        }

		/* General Setup */
		public void Initialize()
		{
			/* Set Talon Status Frame Rates */
			_tal.SetStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 10);    //Send updates every 10ms instead of 10ms
			_tal.SetSensorPhase(true);                                              //reversed sensor

			/* Configure Talon Tach Limit Switches to be Normally Closed */
			_tal.ConfigForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
			_tal.ConfigReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);

			/* Configure Sensor */
			_tal.ConfigSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

			/* Clear Position on Reverse Limit Switch */
			_tal.ConfigClearPositionOnLimitR(false, 30);	//Enable to Zero Position on Limit Switch, Can be used to Home/Set Postions for Software Limit Switches
			_tal.ConfigClearPositionOnLimitF(false, 30);
		}

		/** Setup for PercentOutput */
        private void SetupMotorOutput()
        {
			/* Update Control Mode */
            _controlMode = ControlMode.PercentOutput;
			/* Configure Open Loop Ramp Rate */
			_tal.ConfigOpenloopRamp(1.0f);
        }

		/** Set for PercentOutput */
        public void SetPercentOutput(float percentOutput)
        {
			/* Cap output to 20% */
            percentOutput = CTRE.Phoenix.Util.Cap(percentOutput, 0.25f);
			/* Call Percent Output Setup on Control Mode switch */
			if (_controlMode != ControlMode.PercentOutput) { SetupMotorOutput(); }
			/* Perform Percent Output */
			_gearBox.Set(_controlMode, percentOutput);
			SensorCollection _collection = _tal.GetSensorCollection();
			int value = 0;
			_collection.GetQuadraturePosition(out value);
			Debug.Print("Pos: " + value);
        }

		/** Get Limit switches' fault from Talon */
		public bool isReverseHardwareLimitAsserted
		{
			get
			{
				Faults _talFaults = new Faults();
				_tal.GetFaults(_talFaults);
				return _talFaults.ReverseLimitSwitch;
			}
		}
		public bool isForwardHardwareLimitAsserted
		{
			get
			{
				Faults _talFaults = new Faults();
				_tal.GetFaults(_talFaults);
				return _talFaults.ForwardLimitSwitch;
			}
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
