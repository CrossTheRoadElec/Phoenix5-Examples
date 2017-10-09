#include <Tasks/TaskPWMmotorController.h>
#include "ctre/phoenix/LinearInterpolation.h"
#include "Platform/Platform.h"

TaskPWMmotorController::~TaskPWMmotorController() {}
/* ILoopable */
void TaskPWMmotorController::OnStart() {
	/* If already running, do nothing */
	if(_running)
		return;

	/* Start transmitting neutral signal */
    _percentOutput = 0;
    Hardware::canifier->SetPWMOutput(Constants::MotorControllerCh, 0);
    Hardware::canifier->EnablePWMOutput(Constants::MotorControllerCh, true);

    /* Task is now running */
    _running = true;
}
void TaskPWMmotorController::OnStop() {
	/* Stop transmitting PWM */
	Hardware::canifier->EnablePWMOutput(Constants::MotorControllerCh, false);

	/* Task has stopped, now take note */
	_running = false;
}
bool TaskPWMmotorController::IsDone() {
	return false;
}
void TaskPWMmotorController::OnLoop(){
    /* just grab three axis and direct control the components */
    float axis = Hardware::gamepad->GetRawAxis(Constants::Gamepad_y);
    /* scale to typical PWM widths */
    float pulseUs = CTRE::LinearInterpolation::Calculate(axis, -1, 1000, +1, 2000); /* [-1,+1] => [1000,2000]us */
    /* scale to period */
    float periodUs = 4200; // hard-coded for now, this will be settable in future firmware update.
    _percentOutput = pulseUs / periodUs;
    /* set it */
    Hardware::canifier->SetPWMOutput(Constants::MotorControllerCh, _percentOutput);
}
