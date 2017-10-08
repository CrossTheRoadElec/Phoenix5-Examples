/**
 * Task manageing the CANifier outputs to the LED strip.
 */
using CTRE;
using Platform;

public class TaskHSV : CTRE.Tasking.ILoopable
{
    public float Hue { get; set; }
    public float Saturation { get; set; }
    public float Value { get; set; }

    private float _r, _g, _b;

    private CTRE.Signals.MovingAverage _averageR = new CTRE.Signals.MovingAverage(10);
    private CTRE.Signals.MovingAverage _averageG = new CTRE.Signals.MovingAverage(10);
    private CTRE.Signals.MovingAverage _averageB = new CTRE.Signals.MovingAverage(10);

    public void OnLoop()
    {
        if (Saturation > 1) { Saturation = 1; }
        if (Saturation < 0)
            Saturation = 0;

        if (Value > 1)
            Value = 1;
        if (Value < 0)
            Value = 0;

        /* convert to rgb */
        HsvToRgb.Convert(Hue, Saturation, Value, out _r, out _g, out _b);

        _r = _averageR.Process(_r);
        _g = _averageG.Process(_g);
        _b = _averageB.Process(_b);

        /* update CANifier's LED strip */
        Platform.Hardware.canifier.SetLEDOutput(_r, CTRE.CANifier.LEDChannel.LEDChannelA);
        Platform.Hardware.canifier.SetLEDOutput(_g, CTRE.CANifier.LEDChannel.LEDChannelB);
        Platform.Hardware.canifier.SetLEDOutput(_b, CTRE.CANifier.LEDChannel.LEDChannelC);
    }

    public override string ToString()
    {
        return "HSV_ControlLedStrip:" +  _r + ":" + _g + ":" + _b;
    }

    public void OnStart() { }
    public void OnStop() { }
    public bool IsDone() { return false; }
}
