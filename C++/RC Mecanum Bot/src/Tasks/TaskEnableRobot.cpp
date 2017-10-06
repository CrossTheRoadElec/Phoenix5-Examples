#include "Tasks/TaskEnableRobot.h"
#include "Platform/Hardware.h"
#include "Platform/Schedulers.h"
#include "Platform/Tasks.h"

TaskEnableRobot::~TaskEnableRobot() {}
/* ILoopable */
void TaskEnableRobot::OnStart(){
}
void TaskEnableRobot::OnStop(){
}
bool TaskEnableRobot::IsDone(){
	return false;
}
void TaskEnableRobot::OnLoop(){
	if(Hardware::Futuba3Ch->CurrentStatus == CTRE::RCRadio3Ch::Status::Okay){
		Schedulers::PeriodicTasks->Stop(Tasks::TeleopDriveWithGamepad);
		Schedulers::PeriodicTasks->Start(Tasks::TeleopDriveWithRC);
	}else{
		/* Assume the controller is connected, as there is no method to check if there is a connection */
		Schedulers::PeriodicTasks->Start(Tasks::TeleopDriveWithGamepad);
		Schedulers::PeriodicTasks->Stop(Tasks::TeleopDriveWithRC);
	}
}

