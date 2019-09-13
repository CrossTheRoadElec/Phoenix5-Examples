/**
 * Read Logitech game controller and direct control the ARM's motor output with left stick.
 */
using Platform;

public class TaskDirectControlArm : CTRE.Phoenix.Tasking.ILoopable
{
    public void OnLoop()
    {
        float y = +1 * Hardware.gamepad.GetAxis(1); // Ensure Positive is forward, negative is reverse

        CTRE.Phoenix.Util.Deadband(ref y);

        Subsystems.Arm.SetPercentOutput(y);

        /* if Talon was reset, redo config.  This is generally not necessary */
        if(Subsystems.Arm.MotorController.HasResetOccured())
        {
            Subsystems.Arm.Setup();
        }
    }

    public bool IsDone() { return false; }

    public void OnStart() { }

    public void OnStop() {

        Subsystems.Arm.Stop();

    }
}
