//software pid Pigeon heading => talon (PercentOutput, Voltage).
using System;
using Microsoft.SPOT;
using CTRE.Phoenix.Tasking;
using CTRE.Phoenix.MotorControl;
using CTRE.Phoenix.Sensors;
using CTRE.Phoenix.Drive;
using CTRE.Phoenix;
using CTRE.Phoenix.Motion;


namespace CTRE.Motion
{
    public class ServoHoldHeadingWithImu
    {
        PigeonIMU _pidgey;
        IDrivetrain _driveTrain;
        Styles.BasicStyle _selectedStyle;
        ServoParameters _servoParams = new ServoParameters();

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
        public ServoHoldHeadingWithImu(PigeonIMU pigeonImu, IDrivetrain driveTrain, Styles.BasicStyle selectedStyle, ServoParameters parameters, float targetHeading, float maxOutput)
        {
            _pidgey = pigeonImu;
            _driveTrain = driveTrain;
            _selectedStyle = selectedStyle;
            _targetHeading = targetHeading;
            _servoParams = parameters;
            _maxOutput = maxOutput;
        }

        /** Go Straight using the IMU */
        public ServoHoldHeadingWithImu(PigeonIMU pigeonImu, IDrivetrain driveTrain, Styles.BasicStyle selectedStyle)
        {
            _pidgey = pigeonImu;
            _driveTrain = driveTrain;
            _selectedStyle = selectedStyle;
        }


        /** Return the heading from the Pigeon*/
        public float GetImuHeading()
        {
            _pidgey.GetYawPitchRoll(YPR);
            return YPR[0];
        }
        /**
         * Uses forward, strafe, and turn (Mecanum drive)
         * 
         * @param   forward     Y direction of robot
         * @param   strafe      X direction of robot
         */
        public void Set(Styles.BasicStyle mode, float forward, float strafe)
        {
            Enable(true);
            GoStraight(forward, _targetHeading);
        }

        private void GoStraight(float Y, float targetHeading)
        {
            if (_enableCompensating == false)
            {
                /* calling app has turned this off */
                _isCompensating = false;
            }
            else if(_pidgey.GetState() != PigeonState.Ready)
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
            _driveTrain.Set(_selectedStyle, Y, x_correction);
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
        private void Enable(bool enable)
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

        public void Disable()
        {
            Enable(false);
        }
    }
}
