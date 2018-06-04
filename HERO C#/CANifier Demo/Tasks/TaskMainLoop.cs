/**
 * If a USB Game controller (Logitech wireless, Xbox 360, similar) is connected use that for control
 * If no gamepad is present or user flipped D-X Switch to disable, disable the robot.
 * Additionally the shoulder buttons switch the Arm and Wheel between closed loop and open loop modes.
 **/
using CTRE.Phoenix.Tasking;
using CTRE.Phoenix;
using Platform;

public class TaskMainLoop : ILoopable
{
    public void OnLoop()
    {
        bool gamepadOk = false;

        /* keep feeding watchdog to enable motors */
        if (Hardware.gamepad.GetConnectionStatus() == UsbDeviceConnection.Connected)
        {
            /* keep talons enabled */
            CTRE.Phoenix.Watchdog.Feed();
            /* take note if gamepad is present */
            gamepadOk = true;
        }

        if (gamepadOk)
        {
            Platform.Schedulers.PeriodicTasks.Start(Platform.Tasks.taskPWMmotorController);
        }
        else
        {
            Platform.Schedulers.PeriodicTasks.Stop(Platform.Tasks.taskPWMmotorController);
        }


        if (!gamepadOk || Hardware.gamepad.GetButton(6))
        {
            /* no gamepad present OR user is holding R shoulder btn. Just roll thru color wheel*/
            Platform.Schedulers.PeriodicTasks.Start(Platform.Tasks.taskAnimateLEDStrip);
            Platform.Schedulers.PeriodicTasks.Stop(Platform.Tasks.taskDirectControlArm);
            Platform.Schedulers.PeriodicTasks.Stop(Platform.Tasks.taskLIDAR_ControlLEDStrip);
        }
        else if (Hardware.gamepad.GetButton(5))
        {
            /* let user control LED with sticks */
            Platform.Schedulers.PeriodicTasks.Stop(Platform.Tasks.taskAnimateLEDStrip);
            Platform.Schedulers.PeriodicTasks.Start(Platform.Tasks.taskDirectControlArm);
            Platform.Schedulers.PeriodicTasks.Stop(Platform.Tasks.taskLIDAR_ControlLEDStrip);

            Platform.Schedulers.PeriodicTasks.Start(Platform.Tasks.taskMeasurePulseSensors);
        }
        else if (Hardware.gamepad.GetButton(7))
        {
            Platform.Schedulers.PeriodicTasks.Stop(Platform.Tasks.taskAnimateLEDStrip);
            Platform.Schedulers.PeriodicTasks.Stop(Platform.Tasks.taskDirectControlArm);
            Platform.Schedulers.PeriodicTasks.Start(Platform.Tasks.taskLIDAR_ControlLEDStrip);

            Platform.Schedulers.PeriodicTasks.Start(Platform.Tasks.taskMeasurePulseSensors);
        }
    }

    public void OnStart()
    {
        /* default to LED strip animation */
        Schedulers.PeriodicTasks.Start(Platform.Tasks.taskAnimateLEDStrip);
        Schedulers.PeriodicTasks.Stop(Platform.Tasks.taskDirectControlArm);
        Schedulers.PeriodicTasks.Stop(Platform.Tasks.taskLIDAR_ControlLEDStrip);
    }
    public void OnStop() { }
    public bool IsDone() { return false; }
}
