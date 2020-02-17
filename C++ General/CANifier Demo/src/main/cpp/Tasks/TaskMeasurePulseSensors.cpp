#include "Tasks/TaskMeasurePulseSensors.h"
#include "ctre/Phoenix.h"
#include "FRC_NetworkCommunication/CANSessionMux.h"
#include "Platform/Platform.h"

float TaskMeasurePulseSensors::GetMeasuredPulseWidthsUs(
		CANifier::PWMChannel pwmCh) {
	return _dutyCycleAndPeriods[(int) pwmCh][0];
}
/* Destructor */
TaskMeasurePulseSensors::~TaskMeasurePulseSensors() { }

/* ILoopable */
void TaskMeasurePulseSensors::OnStart() {
}
void TaskMeasurePulseSensors::OnStop() {
}
bool TaskMeasurePulseSensors::IsDone() {
	return false;
}
void TaskMeasurePulseSensors::OnLoop() {
	/* Retrieve PWM from the CANifier connected to our PWM source */
	Hardware::canifier->GetPWMInput(CANifier::PWMChannel::PWMChannel0,
			_dutyCycleAndPeriods[0]);
	Hardware::canifier->GetPWMInput(CANifier::PWMChannel::PWMChannel1,
			_dutyCycleAndPeriods[1]);
	Hardware::canifier->GetPWMInput(CANifier::PWMChannel::PWMChannel2,
			_dutyCycleAndPeriods[2]);
	Hardware::canifier->GetPWMInput(CANifier::PWMChannel::PWMChannel3,
			_dutyCycleAndPeriods[3]);

	/* Send CAN data */
	uint8_t data = 0;
	int status = 0;
	data = (uint) (_dutyCycleAndPeriods[3][0] * 1000);
	FRC_NetworkCommunication_CANSessionMux_sendMessage(0x1E040000, &data, 4, 0,
			&status);

}
