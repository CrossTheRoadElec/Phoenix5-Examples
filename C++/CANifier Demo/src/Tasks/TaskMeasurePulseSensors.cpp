#include <Tasks/TaskMeasurePulseSensors.h>
#include "FRC_NetworkCommunication/CANSessionMux.h"
#include "Platform/Platform.h"

float TaskMeasurePulseSensors::GetMeasuredPulseWidthsUs(CTRE::CANifier::PWMChannel pwmCh){
	return _dutyCycleAndPeriods[(int)pwmCh][0];
}
TaskMeasurePulseSensors::~TaskMeasurePulseSensors() {}
/* ILoopable */
void TaskMeasurePulseSensors::OnStart() {
}
void TaskMeasurePulseSensors::OnStop() {
}
bool TaskMeasurePulseSensors::IsDone() {
	return false;
}
void TaskMeasurePulseSensors::OnLoop(){
    Hardware::canifier->GetPWMInput(CTRE::CANifier::PWMChannel::PWMChannel0, _dutyCycleAndPeriods[0]);
    Hardware::canifier->GetPWMInput(CTRE::CANifier::PWMChannel::PWMChannel1, _dutyCycleAndPeriods[1]);
    Hardware::canifier->GetPWMInput(CTRE::CANifier::PWMChannel::PWMChannel2, _dutyCycleAndPeriods[2]);
    Hardware::canifier->GetPWMInput(CTRE::CANifier::PWMChannel::PWMChannel3, _dutyCycleAndPeriods[3]);

    uint8_t data = 0;
    data = (uint)(_dutyCycleAndPeriods[3][0] * 1000);
    //Have to use the other send?
    int status = 0;
	FRC_NetworkCommunication_CANSessionMux_sendMessage(0x1E040000, &data, 4, 0, &status);

}
