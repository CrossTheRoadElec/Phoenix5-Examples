/**
 * If a USB Game controller (Logitech wireless, Xbox 360, similar) is connected use that for control
 * If a PWM RC Futaba receiver is present, use that.
 * Otherwise disable the HERO robot controller, which disables the Talons.
 * 
 * Note the way this task can stop/start other tasks, allowing for subsystem level control.
 */
using CTRE.Phoenix.Tasking;
using HERO_Mecanum_Drive_Example.Platform;

namespace HERO_Mecanum_Drive_Example
{
    public class TaskEnableRobot : ILoopable
    {
        public bool IsDone()
        {
            return false;
        }

        public void OnLoop()
        {
            /* keep feeding watchdog to enable motors */
            if (Hardware.gamepad.GetConnectionStatus() == CTRE.Phoenix.UsbDeviceConnection.Connected)
            {
				CTRE.Phoenix.Watchdog.Feed();

                Platform.Schedulers.PeriodicTasks.Stop(Platform.Tasks.TeleopDriveWithRC);
                Platform.Schedulers.PeriodicTasks.Start(Platform.Tasks.TeleopDriveWithXbox);
            }
            else if (Hardware.Futaba3Ch.CurrentStatus == CTRE.Phoenix.RCRadio3Ch.Status.Ok)
            {
                CTRE.Phoenix.Watchdog.Feed();

                /* no gamepad, assume RC radio is being used.  If its not, channels will read zero */
                Platform.Schedulers.PeriodicTasks.Stop(Platform.Tasks.TeleopDriveWithXbox);
                Platform.Schedulers.PeriodicTasks.Start(Platform.Tasks.TeleopDriveWithRC);
            }
            else
            {
                Platform.Schedulers.PeriodicTasks.Stop(Platform.Tasks.TeleopDriveWithXbox);
                Platform.Schedulers.PeriodicTasks.Stop(Platform.Tasks.TeleopDriveWithRC);
                Hardware.drivetrain.Set(CTRE.Phoenix.Drive.Styles.BasicStyle.PercentOutput, 0, 0);
            }
        }

        public void OnStart()
        {
        }

        public void OnStop()
        {
        }
    }
}
