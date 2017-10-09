/** changes to the framework class below will be merged into Phoenix Framework */
using System;
using Microsoft.SPOT;
using CTRE.MotorControllers;

namespace CTRE.Mechanical
{
    public class VersaPlanetaryWithMagEnc : SensoredGearbox
    {
        /**
         * @param talon that is connected to VP.
         */
        public VersaPlanetaryWithMagEnc(TalonSrx talon)
            : base(4096f, talon, SmartMotorController.FeedbackDevice.QuadEncoder)
        {
        }
        /**
         * @param ratioBetweenSensorAndGearedOutput If there is a redunction between sensor
         * slice and geared output, enter the ratio (or product of all slices).
         * 
         * @param talon that is connected to VP.
         */
        public VersaPlanetaryWithMagEnc(float ratioBetweenSensorAndGearedOutput, TalonSrx talon)
            : base(ratioBetweenSensorAndGearedOutput, talon, TalonSrx.FeedbackDevice.CtreMagEncoder_Relative)
        {
        }
    }
}