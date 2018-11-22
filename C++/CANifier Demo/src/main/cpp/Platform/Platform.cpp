#include "Platform/Platform.h"
#include "ctre/Phoenix.h"

/* Hardware */
CANifier* Hardware::canifier = new CANifier(0);
frc::Joystick* Hardware::gamepad = new frc::Joystick(0);

/* Schedulers */
ConcurrentScheduler* Schedulers::PeriodicTasks = new ConcurrentScheduler();

/* Tasks */
TaskAnimateLEDStrip* Tasks::taskAnimateLEDStrip = new TaskAnimateLEDStrip();
TaskDirectControlLEDStrip* Tasks::taskDirectControlArm = new TaskDirectControlLEDStrip();
TaskPWMmotorController* Tasks::taskPWMmotorController = new TaskPWMmotorController();
TaskMeasurePulseSensors* Tasks::taskMeasurePulseSensors = new TaskMeasurePulseSensors();
TaskLIDARControlLEDStrip* Tasks::taskLIDARControlLEDStrip = new TaskLIDARControlLEDStrip();
TaskHSV* Tasks::taskHSVControlLedStrip = new TaskHSV();
TaskMainLoop* Tasks::taskMainLoop = new TaskMainLoop();

/* TaskList */
std::vector<ILoopable*> Tasks::FullList = {
		taskAnimateLEDStrip,
		taskDirectControlArm,
		taskPWMmotorController,
		taskMeasurePulseSensors,
		taskLIDARControlLEDStrip,
		taskHSVControlLedStrip,
		taskMainLoop,
};
