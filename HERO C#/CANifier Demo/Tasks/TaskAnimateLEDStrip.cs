/**
 * Task manageing the CANifier outputs to the LED strip.
 */
using CTRE;

public class TaskAnimateLEDStrip : CTRE.Tasking.ILoopable
{
    /* HSV values, @see https://en.wikipedia.org/wiki/HSL_and_HSV  */
    private float _theta;
    private float _saturation;
    private float _value = 0.05f; /* hardcode the brightness */

    private float _r, _g, _b;

    private bool _running;

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
        HsvToRgb.Convert(_theta, _saturation, _value, out _r, out _g, out _b);

        /* update CANifier's LED strip */
        Platform.Hardware.canifier.SetLEDOutput(_r, CTRE.CANifier.LEDChannel.LEDChannelA);
        Platform.Hardware.canifier.SetLEDOutput(_g, CTRE.CANifier.LEDChannel.LEDChannelB);
        Platform.Hardware.canifier.SetLEDOutput(_b, CTRE.CANifier.LEDChannel.LEDChannelC);
    }

    public override string ToString()
    {
        return "AnimateLEDStrip:" + (_running ? "1" : "0") + ":" + _r + ":" + _g + ":" + _b;
    }

    public void OnStart() { _running = true; }
    public void OnStop() { _running = false; }
    public bool IsDone() { return false; }
}
