/**
 * Read game controller and set target WHEEL speeds based on buttons.
 */
using Platform;

public class TaskServoWheelSpeed : CTRE.Phoenix.Tasking.ILoopable
{
    float _target = 0;

    public void OnLoop()
    {
        if (Hardware.gamepad.GetButton(3))
        {
            _target = Constants.SpeedTarget1;
        }
        else if (Hardware.gamepad.GetButton(4))
        {
            _target = Constants.SpeedTarget2;
        }

        Subsystems.Wheel.ServoToSpeed(_target);

        if (Subsystems.Wheel.MotorController.HasResetOccured())
        {
            Subsystems.Wheel.Setup();
        }
    }

    public override string ToString()
    {
        return "WheelServo:" + "T:" + _target + " RPM:" + Subsystems.Wheel.MeasuredSpeed;
    }

    public bool IsDone() { return false; }

    public void OnStart() { }

    public void OnStop()
    {
        Subsystems.Wheel.Stop();
    }
}