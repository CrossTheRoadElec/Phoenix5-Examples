/**
 * Open Loop Wheel Task, Read Logitech game controller and direct control the Wheel's motor output
 * with right joystick. Task can be started by pressing left shoulder button.
 * 
 * Speed is measued through Talon Tach connected to Talon SRX 
 * @link http://www.ctr-electronics.com/talon-tach-tachometer-new-limit-switch.html
 */
using CTRE.Phoenix.Tasking;
using Platform;

public class TaskDirectControlWheel : ILoopable
{
    float _percentOut = 0;

    public void OnLoop()
    {
		/* Get right joystick. Posivite is turn right, Negative is turn left */
        float rightStickY = Hardware.gamepad.GetAxis(5);

		/* Deadband joystick value to remove small noise */
        CTRE.Phoenix.Util.Deadband(ref rightStickY);

		/* Store Joystick value into output value and cap with [-1, 1], where Voltage Comp will cap to [-13V, 13V] */
        _percentOut = rightStickY; // [-1,1]
        _percentOut = CTRE.Phoenix.Util.Cap(_percentOut, 1f);

		/* Call Open Loop method and provide percent output */
        Subsystems.Wheel.SetPercentOutput(_percentOut);

		/* If Talon has reset, redo initialization.  This is generally not necessary */
		if (Subsystems.Wheel.MotorController.HasResetOccured())
        {
            Subsystems.Wheel.Initialize();
        }
    }

	/* ILoopables */
    public void OnStart() { }
	public void OnStop()
	{
		Subsystems.Wheel.Stop();
	}
	public bool IsDone() { return false; }
}