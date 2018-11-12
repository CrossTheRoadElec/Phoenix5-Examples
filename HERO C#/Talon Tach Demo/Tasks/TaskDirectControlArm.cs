/**
 * Open Loop Arm Task, Read Logitech game controller and direct control the Arm's motor output
 * with left joystick. Task can be started by pressing left shoulder button.
 * 
 * Talon Tach connected to Talon SRX used as Hardware Limit Switches
 * @link http://www.ctr-electronics.com/talon-tach-tachometer-new-limit-switch.html
 */
using CTRE.Phoenix.Tasking;
using Platform;

public class TaskDirectControlArm : ILoopable
{
    public void OnLoop()
    {
		/* Get left joystick. Posivite is turn left, Negative is turn right */
		float y = -1 * Hardware.gamepad.GetAxis(1); // Ensure Positive is forward, negative is reverse

		/* Deadband joystick value to remove small noise */
		CTRE.Phoenix.Util.Deadband(ref y);

		/* Call Open Loop method and provide percent output */
		Subsystems.Arm.SetPercentOutput(y);

		/* If Talon has reset, redo initialization.  This is generally not necessary */
		if (Subsystems.Arm.MotorController.HasResetOccured())
        {
            Subsystems.Arm.Initialize();
        }
    }

	/* ILoopables */
    public void OnStart() { }
    public void OnStop()
	{
        Subsystems.Arm.Stop();
    }
	public bool IsDone() { return false; }

}
