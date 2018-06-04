/**
 * Read Logitech game controller and direct control the LED strip using CANifier.
 */
using Platform;
using CTRE.Phoenix.Tasking;

public class TaskDirectControlLEDStrip : ILoopable
{
    public void OnLoop()
    {
        /* get an x and y pair */
        float x = Hardware.gamepad.GetAxis(Constants.GamePadAxis_x);
        float y = Hardware.gamepad.GetAxis(Constants.GamePadAxis_y);
        /* calc theta in deg */
        float theta = (float)System.Math.Atan2(x, y) * 180f / (float)System.Math.PI;
        /* take magnitude and cap it at '1'.  This will be our saturation (how far away from white we want to be) */
        float saturation = (float)System.Math.Sqrt(x * x + y * y);
        saturation = CTRE.Phoenix.Util.Cap(saturation, 1);
        /* pick a value of '1', how far away from black we want to be. */
        Platform.Tasks.taskHSV_ControlLedStrip.Hue = theta;
        Platform.Tasks.taskHSV_ControlLedStrip.Saturation = saturation;
        Platform.Tasks.taskHSV_ControlLedStrip.Value = 1; /* scale down for brightness */
    }

    public override string ToString()
    {
        return "ControlLEDStrip:";
    }

    public bool IsDone() { return false; }
    public void OnStart() {  }
    public void OnStop() {}
}
