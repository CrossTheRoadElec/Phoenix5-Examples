//software pid Pigeon heading => talon (PercentOutput, Voltage).
using System;
using Microsoft.SPOT;
using CTRE.Drive;
using CTRE.Tasking;
using CTRE.MotorControllers;

namespace CTRE.Motion
{
    public class ServoHoldHeadingWithImu : ILoopable
    {
        PigeonImu _pidgey;
        IDrivetrain _driveTrain;
        Styles.Basic _selectedStyle;
        ServoParameters _servoParams = new ServoParameters();

        float _Y;
        float _targetHeading;
        float _maxOutput;

        bool _isCompensating = false;
        bool _enableCompensating = false;

        float[] XYZ_Dps = new float[3];
        float[] YPR = new float[3];

        /* Servo parameters */
        public ServoParameters ServoParameters
        {
            get { return _servoParams; }
        }

        /** Go Straight using the IMU */
        public ServoHoldHeadingWithImu(PigeonImu pigeonImu, IDrivetrain driveTrain, Styles.Basic selectedStyle, ServoParameters parameters, float Y, float targetHeading, float maxOutput)
        {
            _pidgey = pigeonImu;
            _driveTrain = driveTrain;
            _selectedStyle = selectedStyle;
            _Y = Y;
            _targetHeading = targetHeading;
            _servoParams = parameters;
            _maxOutput = maxOutput;
        }

        /** Go Straight using the IMU */
        public ServoHoldHeadingWithImu(PigeonImu pigeonImu, IDrivetrain driveTrain, Styles.Basic selectedStyle)
        {
            _pidgey = pigeonImu;
            _driveTrain = driveTrain;
            _selectedStyle = selectedStyle;
        }

        public void Set(float Y)
        {
            _Y = Y;
        }

        /** Return the heading from the Pigeon*/
        public float GetImuHeading()
        {
            _pidgey.GetYawPitchRoll(YPR);
            return YPR[0];
        }

        private void GoStraight(float Y, float targetHeading)
        {
            if (_enableCompensating == false)
            {
                /* calling app has turned this off */
                _isCompensating = false;
            }
            else if(_pidgey.GetState() != PigeonImu.PigeonState.Ready)
            {
                /* pigeon is not present on bus */
                _isCompensating = false;
            }
            else
            {
                /* feature enabled and pigeon is present, we are good to go */
                _isCompensating = true;
            }

            /* if we can compensate do it, otherwise just apply same output on both sides */
            float x_correction;

            if (_isCompensating == false)
            {
                /* apply same to both sides */
                x_correction = 0;
            }
            else
            {
                /* let user know if they have more work to do */
                if (_servoParams.P == 0 && _servoParams.I == 0 && _servoParams.D == 0)
                    Debug.Print("CTR: Servo Go Straight With Imu has no PID values, cannot go straight");

                /* Grab current heading */
                float currentHeading = GetImuHeading();

                /* Grab angular rate from the pigeon */
                _pidgey.GetRawGyro(XYZ_Dps);
                float currentAngularRate = XYZ_Dps[2];

                /* Heading PID */
                float headingError = targetHeading - currentHeading;
                float X = (headingError) * _servoParams.P - (currentAngularRate) * _servoParams.D;
                X = Util.Cap(X, _maxOutput);
                x_correction = -X;

            }

            /* Select control mode based on selected style */
            switch (_selectedStyle)
            {
                case Styles.Basic.PercentOutput:
                    _driveTrain.Set(Styles.Basic.PercentOutput, Y, x_correction);
                    break;
                case Styles.Basic.Voltage:
                    _driveTrain.Set(Styles.Basic.Voltage, Y, x_correction);
                    break;
            }
        }

        public bool IsCompensating
        {
            get
            {
                return _isCompensating;
            }
        }

        /**
         * Caller can directly enable/disable the feaure if a scheduler is not in use 
         */
        public void Enable(bool enable)
        {
            if (enable == false)
            {
                /* turn it off */
                _enableCompensating = false;
            }
            else if (_enableCompensating == true)
            {
                /* feature is already on, do nothing */
            }
            else
            {
                /* caller wants it on, lock the heading in */
                _targetHeading = GetImuHeading();
                _enableCompensating = true;
            }
        }

        public bool IsEnabled
        {
            get
            {
                return _enableCompensating;
            }
        }

        /** ILoopable */
        public void OnStart()
        {
            Enable(true);
            ///* resync to current heading */
            //_targetHeading = GetImuHeading();
            //_enableCompensating = true;
        }

        public void OnStop()
        {
            Enable(false);
            //_driveTrain.Set(Styles.Basic.PercentOutput, 0, 0);
            //_isCompensating = false;
            //_enableCompensating = false;
        }

        public bool IsDone()
        {
            /* this servo is never done, this is a continuous motion */
            return false;
        }

        public void OnLoop()
        {
            GoStraight(_Y, _targetHeading);
        }
    }
}
