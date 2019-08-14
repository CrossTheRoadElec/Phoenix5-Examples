/**
 * If a USB Game controller (Logitech wireless, Xbox 360, similar) is connected use that for control
 * If no gamepad is present or user flipped D-X Switch to disable, disable the robot.
 * Additionally the shoulder buttons switch the Arm and Wheel between closed loop and open loop modes.
 **/
using CTRE.Phoenix.Tasking;
using Platform;

public class TaskEnableRobot : ILoopable
{

    public void OnLoop()
    {
        bool gamepadOk = false;
        /* keep feeding watchdog to enable motors */
        if (Hardware.gamepad.GetConnectionStatus() == CTRE.Phoenix.UsbDeviceConnection.Connected)
        {
			/* keep talons enabled */
			CTRE.Phoenix.Watchdog.Feed();
            /* take note if gamepad is present */
            gamepadOk = true;
        }


        if (gamepadOk == false)
        {
            /* no gamepad?  stop all tasks */
            Platform.Schedulers.PeriodicTasks.Stop(Platform.Tasks.taskDirectControlArm);
            Platform.Schedulers.PeriodicTasks.Stop(Platform.Tasks.taskServoArmPos);
            Platform.Schedulers.PeriodicTasks.Stop(Platform.Tasks.taskDirectControlWheel);
            Platform.Schedulers.PeriodicTasks.Stop(Platform.Tasks.taskServoWheelSpeed);

        }
        else if (Hardware.gamepad.GetButton(5))
        {
            /* left shoulder means use the open loop tasks */
            Platform.Schedulers.PeriodicTasks.Start(Platform.Tasks.taskDirectControlArm);
            Platform.Schedulers.PeriodicTasks.Start(Platform.Tasks.taskDirectControlWheel);
            Platform.Schedulers.PeriodicTasks.Stop(Platform.Tasks.taskServoArmPos);
            Platform.Schedulers.PeriodicTasks.Stop(Platform.Tasks.taskServoWheelSpeed);
        }
        else if (Hardware.gamepad.GetButton(6))
        {
            /* right  shoulder means use the closed loop tasks */
            Platform.Schedulers.PeriodicTasks.Stop(Platform.Tasks.taskDirectControlArm);
            Platform.Schedulers.PeriodicTasks.Stop(Platform.Tasks.taskDirectControlWheel);
            Platform.Schedulers.PeriodicTasks.Start(Platform.Tasks.taskServoArmPos);
            Platform.Schedulers.PeriodicTasks.Start(Platform.Tasks.taskServoWheelSpeed);
        }
    }

    public void OnStart() { }
    public void OnStop() { }
    public bool IsDone() { return false; }
}
