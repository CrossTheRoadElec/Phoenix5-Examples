/**
 * Closed Loop Arm Task, Read Logitech game controller and drive arm to one of two Limit Switches
 * Button X and Button A allow Servos to two different postions
 * Press the Right should button to start this task
 * 
 * TODO: Redesign Task to use Postion Closed Loop mode to servo to various positions between Hardware Limit Switches
 * Currently it allows the user to uset the two buttons to drive to one of the two Hardware Limit switches (Not Smart)
 * 
 */
using CTRE.Phoenix.Tasking;
using Platform;

public class TaskServoArmPos : ILoopable
{
	/* Track Target based on button press */
    float _target = Constants.Target1;		// Not Used
	bool _bTravelForward = false;

	/* Drive Arm down slowly until Hardware Limit has been hit */
	public void OnStart()
	{
		/* Update Target with default Target */
		_target = Constants.Target1;

		_bTravelForward = false;
	}

	public void OnLoop()
	{
		/* Travel Back and Forth between two Hardware Limit Switches (Talon Tach) */
		if (Hardware.gamepad.GetButton(1)) { _bTravelForward = true; }
		else if (Hardware.gamepad.GetButton(2)) { _bTravelForward = false;  }

		if (!_bTravelForward)
		{
			if (Subsystems.Arm.isReverseHardwareLimitAsserted)
			{
				/* Do Nothing */
			}
			else
			{
				/* As of right now, drive down slowly */
				Subsystems.Arm.SetPercentOutput(-0.20f);
			}
		}
		else
		{
			if (Subsystems.Arm.isForwardHardwareLimitAsserted)
			{
				/* Do Nothing */
			}
			else
			{
				/* As of right now, drive down slowly */
				Subsystems.Arm.SetPercentOutput(0.20f);
			}
		}

		/* If Talon has reset, redo initialization.  This is generally not necessary */
		if (Subsystems.Arm.MotorController.HasResetOccured())
		{
			Subsystems.Arm.Initialize();
		}
	}

	public void OnStop()
	{
		Subsystems.Arm.Stop();
	}
	public bool IsDone() { return false; }


	public override string ToString()
    {
        return "T:" + _target + "P:" + Hardware.ArmGearBox.GetPosition();
    }
}