/** changes to the framework class below will be merged into Phoenix Framework */
using System;
using Microsoft.SPOT;
using CTRE.Phoenix;
namespace CTRE.Motion
{
    public class ServoParameters
    {
        public float P = 0;
        public float I = 0;
        public float D = 0;
        public float F = 0;
        public float maxOut = float.MaxValue;
        public float nominalOut = 0;
        public float IZone = 0;
        public float IMax = float.MaxValue;
        public float allowedError = 0;
        public float durationThreshold = 0;

        private float IAccum = 0;
        private Stopwatch _st = new CTRE.Phoenix.Stopwatch();


        public void ResetIntegralAccum()
        {
            IAccum = 0;
        }
        public float GetIntegralAccum()
        {
            return IAccum;
        }
        //Sensor Derivative added for use with Pigeon get raw gyro and Talon get velocity measurement
        public float PID(float targetValue, float sensorValue, float sensorDerivative)
        {
            float error = targetValue - sensorValue;
            //First call to PID means the stopwatch is guaranteed to have started
            IAccum += I * error;
            if (IAccum > IMax) IAccum = IMax;
            if (IAccum < -IMax) IAccum = -IMax;
            if (System.Math.Abs(error) > IZone) ResetIntegralAccum();

            float fout = (P * error) + IAccum - (D * sensorDerivative);
            if (System.Math.Abs(error) > allowedError)
            {
                if (fout > maxOut) { fout = maxOut; }
                if (fout < -maxOut) { fout = -maxOut; }
                if (fout < nominalOut && fout > 0) fout = nominalOut;
                if (fout > -nominalOut && fout < 0) fout = -nominalOut;

                fout += F * targetValue;
                _st.Start();
            }
            else
            {
                fout = 0;
            }
            return fout;
        }
        //Stopwatch time has to be greater than specified time plus one loop cycle to ensure it's not just lag from the stopwatch
        public bool IsDone()
        {
            return _st.Duration > (durationThreshold);
        }
    }
}
