/**
 * All robot platforms have common features that should be listed below.
 * 
 * Typical examples are...
 * - System wide constants that make configuration of your robot simple
 * - List of all hardware components including gearboxes, linkages, motor controllers, etc.
 * - Schedulers to encoder list of manuevers or general round robin tasks.
 * - Subsystems, class containing custom implementations of actuator-sensor-control or high level implementatoin of multi-motor/multi-sensors.
 *   Servo* classes in Phoenix framework will likely replace this for most use cases.
 * - Tasks (commands) meant to executed periodically or in a specific order.  
 *   Tasks can be stopped/started by other tasks or high level logic via the global scheduler(s).
 * 
 * These groups can be split up into multiple modules, or in one Platform file.
 * 
 * Long term there will likely be a configurable GUI in Phoenix Framework to create robot applciations in this format (in multiple languages).
 */

using CTRE.Phoenix.MotorControl.CAN;
using CTRE.Phoenix.Controller;
using CTRE.Phoenix.Tasking;
using CTRE.Phoenix;

namespace Platform
{
    /* System wide constants.  Use 'public static' because these are single objects. */
    public static class Constants
    {
        public const float GamepadDeadband = 0.03f;

        public const int GamePadAxis_x = 0;
        public const int GamePadAxis_y = 1;
        public const int GamePadAxis_red = 0;
        public const int GamePadAxis_green = 1;
        public const int GamePadAxis_blue = 5;

        public const CANifier.PWMChannel kMotorControllerCh = CANifier.PWMChannel.PWMChannel2;
    }

    /* The hardware objects.  Use 'public static' because these are single objects. */
    public static class Hardware
    {
        public static CANifier canifier = new CANifier(0);

        public static GameController gamepad = new GameController(UsbHostDevice.GetInstance(0), 0);
    }

    public static class Schedulers
    {
        /* the schedulers.  Minimally you will likely want one periodic scheduler to run the normal tasks.
         * Additional schedulers could be ConsecutiveSchedulers for entire autonomous movements or preconfigured manuevers.
         * Use 'public static' because these are single objects. */

        public static ConcurrentScheduler PeriodicTasks = new ConcurrentScheduler(10);
    }

    public static class Tasks
    {
        /* Subsystem tasks.  A task could be all the functionality of a substem like a robot arm, 
         * or could be an individual action appy to a subsytem, such as up reading a gamepad and applying it to the drivcetrain.
         * Use 'public static' because these are single objects. 
         */

        public static TaskAnimateLEDStrip taskAnimateLEDStrip = new TaskAnimateLEDStrip();
        public static TaskDirectControlLEDStrip taskDirectControlArm = new TaskDirectControlLEDStrip();
        public static TaskPWMmotorController taskPWMmotorController = new TaskPWMmotorController();
        public static TaskMeasurePulseSensors taskMeasurePulseSensors = new TaskMeasurePulseSensors();
        public static TaskLIDAR_ControlLEDStrip taskLIDAR_ControlLEDStrip = new TaskLIDAR_ControlLEDStrip();
        public static TaskHSV taskHSV_ControlLedStrip = new TaskHSV();

        

        public static TaskMainLoop taskMainLoop = new TaskMainLoop();


        /* insert all Tasks below in the Full List so they get auto inserted, see Program.cs to see how this works.*/
        public static ILoopable[] FullList = {
            taskAnimateLEDStrip,
            taskDirectControlArm,
            taskPWMmotorController,
            taskMeasurePulseSensors,
            taskLIDAR_ControlLEDStrip,
            taskHSV_ControlLedStrip,
            taskMainLoop,
        };
    }
}
