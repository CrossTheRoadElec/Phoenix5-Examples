/**
 * Read Logitech game controller and direct control the LED strip using CANifier.
 */
using Platform;

public class TaskDirectControlLEDStrip : CTRE.Tasking.ILoopable
{
    float red;
    float grn;
    float blu;
    bool running = false;

    public void OnLoop()
    {
        if (true)
        {
            /* get an x and y pair */
            float x = Hardware.gamepad.GetAxis(Constants.GamePadAxis_x);
            float y = Hardware.gamepad.GetAxis(Constants.GamePadAxis_y);
            /* deadband the sticks */
            CTRE.Util.Deadband(ref x, Constants.GamepadDeadband);
            CTRE.Util.Deadband(ref y, Constants.GamepadDeadband);
            /* calc theta in deg */
            float theta = (float)System.Math.Atan2(x, y) * 180f / (float)System.Math.PI;
            /* take magnitude and cap it at '1'.  This will be our saturation (how far away from white we want to be) */
            float saturation = (float)System.Math.Sqrt(x * x + y * y);
            saturation = CTRE.Util.Cap(saturation, 1);
            /* pick a value of '1', how far away from black we want to be. */
            float value = 1; /* scale down for brightness */
            /* calc RGB for this target HSV */
            CTRE.HsvToRgb.Convert(theta, saturation, value, out red, out grn, out blu);
        }
        else
        {
            /* just grab three axis and direct control the components */
            red = Hardware.gamepad.GetAxis(Constants.GamePadAxis_red);
            grn = Hardware.gamepad.GetAxis(Constants.GamePadAxis_green);
            blu = Hardware.gamepad.GetAxis(Constants.GamePadAxis_blue);
            /* deadband the sticks */
            CTRE.Util.Deadband(ref red, Constants.GamepadDeadband);
            CTRE.Util.Deadband(ref grn, Constants.GamepadDeadband);
            CTRE.Util.Deadband(ref blu, Constants.GamepadDeadband);
        }
        /* update CANifier*/
        Hardware.canifier.SetLEDOutput(red, CTRE.CANifier.LEDChannel.LEDChannelA);
        Hardware.canifier.SetLEDOutput(grn, CTRE.CANifier.LEDChannel.LEDChannelB);
        Hardware.canifier.SetLEDOutput(blu, CTRE.CANifier.LEDChannel.LEDChannelC);
    }

    public override string ToString()
    {
        return "ControlLEDStrip:" + (running ? "1" : "0") + ":" + red + ":" + grn + ":" + blu;
    }

    public bool IsDone() { return false; }
    public void OnStart() { running = true; }
    public void OnStop() { running = false; }
}
