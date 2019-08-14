/**
 * Read Logitech game controller and direct control the WHEEL's motor output with left stick.
 * 
 * Speed is measued with a Talon Tach and Talon SRX.
 * @link http://www.ctr-electronics.com/talon-tach-tachometer-new-limit-switch.html
 */
using Platform;
using CTRE.Phoenix;

public class TaskDirectControlWheel : CTRE.Phoenix.Tasking.ILoopable
{
    float _percentOut = 0;

    public void OnLoop()
    {
        float rightStickY = Hardware.gamepad.GetAxis(5);  // Ensure Positive is turn-right, negative is turn-left

        CTRE.Phoenix.Util.Deadband(ref rightStickY);

        _percentOut = rightStickY; // [-1,1]
   //     _percentOut = LinearInterpolation.Calculate(_percentOut, -1, -13f, +1, +13f); // scale to [-13V, +13V]
     //   _percentOut = CTRE.Phoenix.Util.Cap(_percentOut, 13f); // cap to 13V

        Subsystems.Wheel.SetPercentOutput(_percentOut);

        /* if Talon was reset, redo config.  This is generally not necessary */
        if (Subsystems.Wheel.MotorController.HasResetOccured())
        {
            Subsystems.Wheel.Setup();
        }
    }

    public bool IsDone() { return false; }

    public void OnStart() { }

    public void OnStop()
    {
        Subsystems.Wheel.Stop();
    }
}