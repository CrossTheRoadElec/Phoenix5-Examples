/**
 * IMPORTANT: This example requires the version of the SDK from the
 * Installer version 5.0.3.2 or higher.  There were several changes
 * and additions to multiple files in the SDK, and these are required
 * for this example
 * 
 * Anything under the "Framework" folder are tenative changes later to be into the Phoenix Framework.
 * 
 * Anything under "Tasks" are typical examples tasks done on a robot platform.
 */
using CTRE.Phoenix.Tasking;
using Microsoft.SPOT;
using Platform;
using System.Threading;

namespace Talon_Tach_Demo
{
    public class Program : CTRE.Phoenix.RobotApplication
    {
        public override void RunForever()
        {
			/* any system wide initializations here */
			Hardware.armTalon.SetSelectedSensorPosition(0,10);
            /* add all the periodic tasks */
            foreach(ILoopable task in Platform.Tasks.FullList)
                Schedulers.PeriodicTasks.Add(task);

            /* immedietely stop a few tasks since they are started by TaskEnableRobot */
            Schedulers.PeriodicTasks.Stop(Tasks.taskDirectControlArm);
            Schedulers.PeriodicTasks.Stop(Tasks.taskDirectControlWheel);
            Schedulers.PeriodicTasks.Stop(Tasks.taskServoArmPos);
            Schedulers.PeriodicTasks.Stop(Tasks.taskServoWheelSpeed);

            /* loop forever */
            while (true)
            {
                Schedulers.PeriodicTasks.Process();

                /* dump some tasks and subsystems into the console output */
                Debug.Print(Platform.Subsystems.Arm.ToString());
                Debug.Print(Platform.Subsystems.Wheel.ToString());
                Debug.Print(Platform.Tasks.taskServoArmPos.ToString());
                Debug.Print(Platform.Tasks.taskServoWheelSpeed.ToString());

                Thread.Sleep(5);
            }
        }
      
        /** ---------------------------- DO NOT MODIFY ----------------------------//
        /** this is what starts your project. All examples will have this common line */
        public static void Main() { Start(new Program()); }
    }
}
