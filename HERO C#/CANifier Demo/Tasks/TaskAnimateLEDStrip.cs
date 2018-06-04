/**
 * Task manageing the CANifier outputs to the LED strip.
 */
using CTRE.Phoenix.Tasking;

public class TaskAnimateLEDStrip : ILoopable
{
    private float _hue;

    public void OnLoop()
    {
        /* just ramp through the outer rim of the HSV color wheel */
        _hue += 1;
        if (_hue >= 360) { _hue = 0; }

        /* update HSV target */
        Platform.Tasks.taskHSV_ControlLedStrip.Hue = _hue;
        Platform.Tasks.taskHSV_ControlLedStrip.Saturation = 1.0f; /* outer rim of HSV color wheel */
        Platform.Tasks.taskHSV_ControlLedStrip.Value = 0.05f; /* hardcode the brightness */
    }

    public override string ToString()
    {
        return "AnimateLEDStrip:" + _hue;
    }

    public void OnStart() {}
    public void OnStop() {}
    public bool IsDone() { return false; }
}
