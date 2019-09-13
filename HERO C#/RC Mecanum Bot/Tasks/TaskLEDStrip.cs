/**
 * Task manageing the CANifier outputs to the LED strip.
 */
using CTRE;
namespace HERO_Mecanum_Drive_Example
{
    public class TaskLEDStrip : CTRE.Phoenix.Tasking.ILoopable
    {
        /* HSV values, @see https://en.wikipedia.org/wiki/HSL_and_HSV  */
        private float _theta;
        private float _saturation;
        private float _value = 0.05f; /* hardcode the brightness */

        public bool IsDone()
        {
            return false;
        }

        public void OnLoop()
        {

                /* just ramp through the outer rim of the HSV color wheel */
                _saturation = 1;
                _theta += 1;
                if (_theta >= 360) { _theta = 0; }
            

            /* push saturation to the outter rim of the HSV color wheel */
            _saturation *= 3.0f;
            if (_saturation > 1) { _saturation = 1; }

            /* convert to rgb */
            float r, g, b;
            HsvToRgb.Convert(_theta, _saturation, _value, out r, out g, out b);

            /* update CANifier's LED strip */
            Platform.Hardware.canifier_LedStrip_RCRADIO.SetLEDOutput(r, CTRE.Phoenix.CANifier.LEDChannel.LEDChannelA);
            Platform.Hardware.canifier_LedStrip_RCRADIO.SetLEDOutput(g, CTRE.Phoenix.CANifier.LEDChannel.LEDChannelB);
            Platform.Hardware.canifier_LedStrip_RCRADIO.SetLEDOutput(b, CTRE.Phoenix.CANifier.LEDChannel.LEDChannelC);
        }

        public void OnStart()
        {
        }

        public void OnStop()
        {
        }
    }
}
