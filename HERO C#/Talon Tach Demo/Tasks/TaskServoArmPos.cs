/**
 * Read game controller and set target ARM positions based on buttons.
 */
using Platform;

public class TaskServoArmPos : CTRE.Phoenix.Tasking.ILoopable
{
    float _target = Constants.Target1;

    public void OnLoop()
    {
        if (Hardware.gamepad.GetButton(1))
        {
            _target = Constants.Target1;
        }
        else if (Hardware.gamepad.GetButton(2))
        {
            _target = Constants.Target2;
        }

        Subsystems.Arm.SetTargetPos(_target);

        if (Subsystems.Arm.MotorController.HasResetOccured())
        {
            Subsystems.Arm.Setup();
        }
    }

    public override string ToString()
    {
        return "T:" + _target + "P:" + Hardware.ArmGearBox.GetPosition();
    }

    public bool IsDone() { return false; }

    public void OnStart()
    {
        _target = Constants.Target1;
    }

    public void OnStop()
    {
        Subsystems.Arm.Stop();
    }
}