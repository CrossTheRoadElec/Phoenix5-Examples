/** changes to the framework class below will be merged into Phoenix Framework */
using System;
using Microsoft.SPOT;
using CTRE.Drive;
using CTRE.MotorControllers;

namespace CTRE.Mechanical
{
    /**
     * Represents a mechanical system whereby a number of motor controllers is connected together via a gearbox.
     * One of the motor controllers must be a Talon SRX, and it must use the feedback connector.
     */
    public class SensoredGearbox : IInvertable
    {
        private float _unitsPerRevolution;
        private float _rotationsPerSensorUnit;

        /* Check to see if we are inverted */
        private bool _isInverted;
        public SmartMotorController _talon;

        public SensoredGearbox(float unitsPerRevolution, SmartMotorController talon, SmartMotorController.FeedbackDevice feedbackDevice)
        {
            _talon = talon;
            _unitsPerRevolution = unitsPerRevolution;
            _rotationsPerSensorUnit = 1f / _unitsPerRevolution;
            _talon.SetFeedbackDevice(feedbackDevice);
        }
        public SensoredGearbox(float unitsPerRevolution, SmartMotorController talon, MotorControllers.IFollower slaveTalon1, SmartMotorController.FeedbackDevice feedbackDevice)
        {
            _talon = talon;
            _unitsPerRevolution = unitsPerRevolution;
            _rotationsPerSensorUnit = 1f / _unitsPerRevolution;
            _talon.SetFeedbackDevice(feedbackDevice);

            slaveTalon1.Follow(talon);
        }
        public SensoredGearbox(float unitsPerRevolution, SmartMotorController talon, MotorControllers.IFollower slaveTalon1, MotorControllers.IFollower slaveTalon2, SmartMotorController.FeedbackDevice feedbackDevice)
        {
            _talon = talon;
            _unitsPerRevolution = unitsPerRevolution;
            _rotationsPerSensorUnit = 1f / _unitsPerRevolution;
            _talon.SetFeedbackDevice(feedbackDevice);

            slaveTalon1.Follow(talon);
            slaveTalon2.Follow(talon);
        }
        public SensoredGearbox(float unitsPerRevolution, SmartMotorController talon, MotorControllers.IFollower slaveTalon1, MotorControllers.IFollower slaveTalon2, MotorControllers.IFollower slaveTalon3, SmartMotorController.FeedbackDevice feedbackDevice)
        {
            _talon = talon;
            _unitsPerRevolution = unitsPerRevolution;
            _rotationsPerSensorUnit = 1f / _unitsPerRevolution;
            _talon.SetFeedbackDevice(feedbackDevice);

            slaveTalon1.Follow(talon);
            slaveTalon2.Follow(talon);
            slaveTalon3.Follow(talon);
        }

        /**
         * @return The geared output position in rotations.
         */
        public float GetPosition()
        {
            return _talon.GetPosition() * _rotationsPerSensorUnit;
        }
        /**
         * @return The geared output velocity in rotations per minute.
         */
        public float GetVelocity()
        {
            return _talon.GetSpeed() * _rotationsPerSensorUnit;
        }

        public void SetCurrentLimit(uint currentLimitAmps, uint timeoutms)
        {
            _talon.SetCurrentLimit(currentLimitAmps, timeoutms);
        }

        public void Set(float output, ControlMode controlMode)
        {
            if (_isInverted)
                output = -output;

            switch (controlMode)
            {
                case ControlMode.kMotionMagic:
                case ControlMode.kPosition:
                case ControlMode.kSpeed:
                    _talon.SetControlMode(controlMode);
                    _talon.Set(output * _unitsPerRevolution);
                    break;
                default:
                    _talon.SetControlMode(controlMode);
                    _talon.Set(output);
                    break;
            }
        }

        ///** Set the control mode of the gearbox */
        //public void SetControlMode(ControlMode mode)
        //{
        //    _talon.SetControlMode(mode);
        //}

        /** Set the limits on the forward and reverse drive */
        public void SetLimits(float forwardLimit, float reverseLimit)
        {
            _talon.ConfigForwardLimit(forwardLimit);
            _talon.ConfigReverseLimit(reverseLimit);
        }
        
        /** Disable limits of the gearbox */
        public void DisableLimits()
        {
            _talon.ConfigLimitMode(SmartMotorController.LimitMode.kLimitMode_SrxDisableSwitchInputs);
        }

        /* IInvertable */
        public void SetInverted(bool invert)
        {
            _isInverted = invert;
        }

        public bool GetInverted()
        {
            return _isInverted;
        }

        public void InvertSensor(bool invert)
        {
            _talon.SetSensorDirection(invert);
        }

        public void SetSensor(float position)
        {
            position *= _unitsPerRevolution;
            _talon.SetPosition(position);
        }

        public void ConfigNominalOutputVoltage(float forwardVoltage, float reverseVoltage)
        {
            _talon.ConfigNominalOutputVoltage(forwardVoltage, reverseVoltage);
        }

        public void ConfigPeakOutputVoltage(float forwardVoltage, float reverseVoltage)
        {
            _talon.ConfigPeakOutputVoltage(forwardVoltage, reverseVoltage);
        }

        public void SetVoltageCompensationRampRate(float RampRate)
        {
            _talon.SetVoltageCompensationRampRate(RampRate);
        }

        public void SetVoltageRampRate(float RampRate)
        {
            _talon.SetVoltageRampRate(RampRate);
        }

        /* IMotionMagical */
        /* Motion Magic stuff */
        public void SetMotionMagicCruiseVelocity(float RPM)
        {
            _talon.SetMotionMagicCruiseVelocity(RPM);
        }

        public void SetMotionMagicAcceleration(float RPM)
        {
            _talon.SetMotionMagicAcceleration(RPM);
        }

        public IMotorController GetMaster()
        {
            return _talon;
        }
    }
}
