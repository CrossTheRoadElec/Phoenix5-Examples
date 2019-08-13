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

namespace CANifier_Demo
{
    public class Program : CTRE.Phoenix.RobotApplication
    {
        public override void RunForever()
        {
			/* any system wide initializations here */
			/* Factory Default all hardware to prevent unexpected behaviour */
			Hardware.canifier.ConfigFactoryDefault();
			/* add all the periodic tasks */
			foreach (ILoopable task in Platform.Tasks.FullList)
                Schedulers.PeriodicTasks.Add(task);

            /* loop forever */
            while (true)
            {
                Schedulers.PeriodicTasks.Process();

                /* dump some tasks and subsystems into the console output */
               // Debug.Print(Platform.Tasks.taskDirectControlArm.ToString() + " " +
                //            Platform.Tasks.taskAnimateLEDStrip.ToString());
                Debug.Print(Platform.Tasks.taskMeasurePulseSensors.ToString());

                Thread.Sleep(5);
            }
        }

        /** ---------------------------- DO NOT MODIFY ----------------------------//
        /** this is what starts your project. All examples will have this common line */
        public static void Main() { Start(new Program()); }
    }
}
