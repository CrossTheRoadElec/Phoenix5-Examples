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
using System.Threading;
using HERO_Mecanum_Drive_Example.Platform;

namespace HERO_Mecanum_Drive_Example
{
    public class Program : CTRE.Phoenix.RobotApplication
    {
        public override void RunForever()
        {
            /* any system wide initializations here */
            Hardware.leftFrnt.SetInverted(true);
            Hardware.leftRear.SetInverted(true);

            /* add all the periodic tasks */
            Schedulers.PeriodicTasks.Add(Platform.Tasks.EnableRobot);
            Schedulers.PeriodicTasks.Add(Platform.Tasks.LEDStrip);
            Schedulers.PeriodicTasks.Add(Platform.Tasks.TeleopDriveWithRC);
            Schedulers.PeriodicTasks.Add(Platform.Tasks.TeleopDriveWithXbox);
            Schedulers.PeriodicTasks.Add(Platform.Tasks.LowBatteryDetect);
            
            /* loop forever */
            while (true)
            {
                Schedulers.PeriodicTasks.Process();

                Hardware.Futaba3Ch.Process();
                
                Microsoft.SPOT.Debug.Print(Hardware.Futaba3Ch.ToString()); /* example of how to print telemetry data */
                
                Thread.Sleep(5);
            }
        }
      
        /** ---------------------------- DO NOT MODIFY ----------------------------//
        /** this is what starts your project. All examples will have this common line */
        public static void Main() { Start(new Program()); }
    }
}
