/**
 * Task manageing the CANifier outputs to the LED strip.
 */
using CTRE;

public class TaskLIDAR_ControlLEDStrip : CTRE.Tasking.ILoopable
{
    public void OnLoop()
    {
        float pulse = Platform.Tasks.taskMeasurePulseSensors.GetMeasuredPulseWidthsUs(CANifier.PWMChannel.PWMChannel3);

        /* scale [0,8000] us to [0,360' Hue Deg */
        float hue = LinearInterpolation.Calculate(pulse, 0f, 0f, 8000f, 360f);

        Platform.Tasks.taskHSV_ControlLedStrip.Hue = hue;
        Platform.Tasks.taskHSV_ControlLedStrip.Saturation = 1;
        Platform.Tasks.taskHSV_ControlLedStrip.Value = 0.05f; /* hardcode the brightness */
    }

    public void OnStart() { }
    public void OnStop() { }
    public bool IsDone() { return false; }
}
