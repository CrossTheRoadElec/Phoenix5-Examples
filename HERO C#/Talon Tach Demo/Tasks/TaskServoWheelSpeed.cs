/**
 * Closed Loop Wheel Task, Read Logitech game controller and drive wheel at one to two preset RPMs (Constants)
 * Button X and Button A allow Servos at two different RPm
 * Press the Right should button to start this task 
 */
using Platform;
using CTRE.Phoenix.Tasking;

public class TaskServoWheelSpeed : ILoopable
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

		/* If Talon has reset, redo initialization.  This is generally not necessary */
		if (Subsystems.Wheel.MotorController.HasResetOccured())
        {
            Subsystems.Wheel.Initialize();
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