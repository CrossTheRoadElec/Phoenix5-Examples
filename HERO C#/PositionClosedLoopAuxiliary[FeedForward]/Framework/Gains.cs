using System;
using Microsoft.SPOT;

namespace PositionClosedLoopAuxiliary.Framework
{
    public class Gains
    {
        public float kP;
        public float kI;
        public float kD;
        public float kF;
        public float kIzone;
        public float kPeakOutput;

        public Gains(float _kP, float _kI, float _kD, float _kF, float _kIzone, float _kPeakOutput)
        {
            kP = _kP;
            kI = _kI;
            kD = _kD;
            kF = _kF;
            kIzone = _kIzone;
            kPeakOutput = _kPeakOutput;
        }
    }
}
