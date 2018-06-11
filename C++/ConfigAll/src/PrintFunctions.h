/*
 * PrintFunctions.h
 *
 *  Created on: Jun 8, 2018
 *      Author: Ryan Greenblatt
 */

#ifndef SRC_PRINTFUNCTIONS_H_
#define SRC_PRINTFUNCTIONS_H_

#include <iostream>
#include <map>
#include <string>
#include "ctre/Phoenix.h"



std::ostream& operator<<(std::ostream& out, const ctre::phoenix::motorcontrol::LimitSwitchSource value){
    const char* s = 0;
    #define PROCESS_VAL(p) case(p): s = #p; break;
	switch(value) {
        PROCESS_VAL(LimitSwitchSource_FeedbackConnector);     
        PROCESS_VAL(LimitSwitchSource_RemoteTalonSRX);     
        PROCESS_VAL(LimitSwitchSource_RemoteCANifier);
        PROCESS_VAL(LimitSwitchSource_Deactivated);
#undef PROCESS_VAL
    }   

    return out << s;
}
std::ostream& operator<<(std::ostream& out, const ctre::phoenix::motorcontrol::RemoteLimitSwitchSource value){
    const char* s = 0;
    #define PROCESS_VAL(p) case(p): s = #p; break;
	switch(value) {
        PROCESS_VAL(RemoteLimitSwitchSource_RemoteTalonSRX);     
        PROCESS_VAL(RemoteLimitSwitchSource_RemoteCANifier);
        PROCESS_VAL(RemoteLimitSwitchSource_Deactivated);
#undef PROCESS_VAL
    }   

    return out << s;
}
std::ostream& operator<<(std::ostream& out, const ctre::phoenix::motorcontrol::LimitSwitchNormal value){
    const char* s = 0;
    #define PROCESS_VAL(p) case(p): s = #p; break;
	switch(value) {
        PROCESS_VAL(LimitSwitchNormal_NormallyOpen);     
        PROCESS_VAL(LimitSwitchNormal_NormallyClosed);
        PROCESS_VAL(LimitSwitchNormal_Disabled);
#undef PROCESS_VAL
    }   

    return out << s;
}

std::ostream& operator<<(std::ostream& out, const ctre::phoenix::motorcontrol::VelocityMeasPeriod value){
    const char* s = 0;
    #define PROCESS_VAL(p) case(p): s = #p; break;
	switch(value) {
        PROCESS_VAL(VelocityMeasPeriod::Period_1Ms);     
        PROCESS_VAL(VelocityMeasPeriod::Period_2Ms);     
        PROCESS_VAL(VelocityMeasPeriod::Period_5Ms);     
        PROCESS_VAL(VelocityMeasPeriod::Period_10Ms);     
        PROCESS_VAL(VelocityMeasPeriod::Period_20Ms);     
        PROCESS_VAL(VelocityMeasPeriod::Period_25Ms);     
        PROCESS_VAL(VelocityMeasPeriod::Period_50Ms);     
        PROCESS_VAL(VelocityMeasPeriod::Period_100Ms);     
#undef PROCESS_VAL
    }   

    return out << s;
}
std::ostream& operator<<(std::ostream& out, const ctre::phoenix::motorcontrol::FeedbackDevice value){
    const char* s = 0;
#define PROCESS_VAL(p) case(p): s = #p; break;
	switch(value) {
        PROCESS_VAL(None); 
        PROCESS_VAL(QuadEncoder); 
        PROCESS_VAL(Analog);     
        PROCESS_VAL(Tachometer);     
        PROCESS_VAL(PulseWidthEncodedPosition);     
        PROCESS_VAL(SensorSum);     
        PROCESS_VAL(SensorDifference);     
        PROCESS_VAL(RemoteSensor0);     
        PROCESS_VAL(RemoteSensor1);     
        PROCESS_VAL(SoftwareEmulatedSensor);     

#undef PROCESS_VAL
    }   

    return out << s;
}
std::ostream& operator<<(std::ostream& out, const ctre::phoenix::motorcontrol::RemoteFeedbackDevice value){
    const char* s = 0;
#define PROCESS_VAL(p) case(p): s = #p; break;
	switch(value) {
        PROCESS_VAL(RemoteFeedbackDevice_None); 
        PROCESS_VAL(SensorSum); 
        PROCESS_VAL(SensorDifference);     
        PROCESS_VAL(RemoteSensor0);     
        PROCESS_VAL(RemoteSensor1);     
        PROCESS_VAL(SoftwareEmulatedSensor);     
#undef PROCESS_VAL
    }   

    return out << s;
}
std::ostream& operator<<(std::ostream& out, const ctre::phoenix::motorcontrol::RemoteSensorSource value){
    const char* s = 0;
#define PROCESS_VAL(p) case(p): s = #p; break;
	switch(value) {
        PROCESS_VAL(RemoteSensorSource::RemoteSensorSource_Off                    );     
        PROCESS_VAL(RemoteSensorSource::RemoteSensorSource_TalonSRX_SelectedSensor);     
        PROCESS_VAL(RemoteSensorSource::RemoteSensorSource_Pigeon_Yaw             );     
        PROCESS_VAL(RemoteSensorSource::RemoteSensorSource_Pigeon_Pitch           );     
        PROCESS_VAL(RemoteSensorSource::RemoteSensorSource_Pigeon_Roll            );     
        PROCESS_VAL(RemoteSensorSource::RemoteSensorSource_CANifier_Quadrature    );     
        PROCESS_VAL(RemoteSensorSource::RemoteSensorSource_CANifier_PWMInput0     );     
        PROCESS_VAL(RemoteSensorSource::RemoteSensorSource_CANifier_PWMInput1     );     
        PROCESS_VAL(RemoteSensorSource::RemoteSensorSource_CANifier_PWMInput2     );     
        PROCESS_VAL(RemoteSensorSource::RemoteSensorSource_CANifier_PWMInput3     );     
        PROCESS_VAL(RemoteSensorSource::RemoteSensorSource_GadgeteerPigeon_Yaw    );     
        PROCESS_VAL(RemoteSensorSource::RemoteSensorSource_GadgeteerPigeon_Pitch  );     
        PROCESS_VAL(RemoteSensorSource::RemoteSensorSource_GadgeteerPigeon_Roll   );     
#undef PROCESS_VAL
    }   

    return out << s;
}


std::ostream& operator<<(std::ostream& out, const ctre::phoenix::motorcontrol::SensorTerm value){
    const char* s = 0;
#define PROCESS_VAL(p) case(p): s = #p; break;
	switch(value) {
        PROCESS_VAL(SensorTerm::SensorTerm_Sum0);     
        PROCESS_VAL(SensorTerm::SensorTerm_Sum1);     
        PROCESS_VAL(SensorTerm::SensorTerm_Diff0);     
        PROCESS_VAL(SensorTerm::SensorTerm_Diff1);     
#undef PROCESS_VAL
    }   

    return out << s;
}



void PrintTalonConfigPart1(ctre::phoenix::motorcontrol::can::TalonSRXConfiguration &config) {

	std::cout << "_talon.openloopRamp = " << config.openloopRamp << std::endl;
	std::cout << "_talon.closedloopRamp = " << config.closedloopRamp << std::endl;
	std::cout << "_talon.peakOutputForward = " << config.peakOutputForward << std::endl;
	std::cout << "_talon.peakOutputReverse = " << config.peakOutputReverse << std::endl;
	std::cout << "_talon.nominalOutputForward = " << config.nominalOutputForward << std::endl;
	std::cout << "_talon.nominalOutputReverse = " << config.nominalOutputReverse << std::endl;
	std::cout << "_talon.neutralDeadband = " << config.neutralDeadband << std::endl;
	std::cout << "_talon.voltageCompSaturation = " << config.voltageCompSaturation << std::endl;
	std::cout << "_talon.voltageMeasurementFilter = " << config.voltageMeasurementFilter << std::endl;
	std::cout << "_talon.velocityMeasurementPeriod = " << config.velocityMeasurementPeriod << std::endl;
	std::cout << "_talon.velocityMeasurementWindow = " << config.velocityMeasurementWindow << std::endl;
	std::cout << "_talon.forwardLimitSwitchDeviceID = " << config.forwardLimitSwitchDeviceID << std::endl;
	std::cout << "_talon.reverseLimitSwitchDeviceID = " << config.reverseLimitSwitchDeviceID << std::endl;
	std::cout << "_talon.forwardLimitSwitchNormal = " << config.forwardLimitSwitchNormal << std::endl;
	std::cout << "_talon.reverseLimitSwitchNormal = " << config.reverseLimitSwitchNormal << std::endl;
	std::cout << "_talon.forwardSoftLimitThreshold = " << config.forwardSoftLimitThreshold << std::endl;
	std::cout << "_talon.reverseSoftLimitThreshold = " << config.reverseSoftLimitThreshold << std::endl;
	std::cout << "_talon.forwardSoftLimitEnable = " << config.forwardSoftLimitEnable << std::endl;
	std::cout << "_talon.reverseSoftLimitEnable = " << config.reverseSoftLimitEnable << std::endl;
}
void PrintTalonConfigPart2(ctre::phoenix::motorcontrol::can::TalonSRXConfiguration &config) {
	std::cout << "_talon.slot_0.kP = " << config.slot_0.kP << std::endl;
	std::cout << "_talon.slot_0.kI = " << config.slot_0.kI << std::endl;
	std::cout << "_talon.slot_0.kD = " << config.slot_0.kD << std::endl;
	std::cout << "_talon.slot_0.kF = " << config.slot_0.kF << std::endl;
	std::cout << "_talon.slot_0.integralZone = " << config.slot_0.integralZone << std::endl;
	std::cout << "_talon.slot_0.allowableClosedloopError = " << config.slot_0.allowableClosedloopError << std::endl;
	std::cout << "_talon.slot_0.maxIntegralAccumulator = " << config.slot_0.maxIntegralAccumulator << std::endl;
	std::cout << "_talon.slot_0.closedLoopPeakOutput = " << config.slot_0.closedLoopPeakOutput << std::endl;
	std::cout << "_talon.slot_0.closedLoopPeriod = " << config.slot_0.closedLoopPeriod << std::endl;
	std::cout << "_talon.slot_1.kP = " << config.slot_1.kP << std::endl;
	std::cout << "_talon.slot_1.kI = " << config.slot_1.kI << std::endl;
	std::cout << "_talon.slot_1.kD = " << config.slot_1.kD << std::endl;
	std::cout << "_talon.slot_1.kF = " << config.slot_1.kF << std::endl;
	std::cout << "_talon.slot_1.integralZone = " << config.slot_1.integralZone << std::endl;
	std::cout << "_talon.slot_1.allowableClosedloopError = " << config.slot_1.allowableClosedloopError << std::endl;
	std::cout << "_talon.slot_1.maxIntegralAccumulator = " << config.slot_1.maxIntegralAccumulator << std::endl;
	std::cout << "_talon.slot_1.closedLoopPeakOutput = " << config.slot_1.closedLoopPeakOutput << std::endl;
	std::cout << "_talon.slot_1.closedLoopPeriod = " << config.slot_1.closedLoopPeriod << std::endl;
	std::cout << "_talon.slot_2.kP = " << config.slot_2.kP << std::endl;
	std::cout << "_talon.slot_2.kI = " << config.slot_2.kI << std::endl;
	std::cout << "_talon.slot_2.kD = " << config.slot_2.kD << std::endl;
	std::cout << "_talon.slot_2.kF = " << config.slot_2.kF << std::endl;
	std::cout << "_talon.slot_2.integralZone = " << config.slot_2.integralZone << std::endl;
	std::cout << "_talon.slot_2.allowableClosedloopError = " << config.slot_2.allowableClosedloopError << std::endl;
	std::cout << "_talon.slot_2.maxIntegralAccumulator = " << config.slot_2.maxIntegralAccumulator << std::endl;
	std::cout << "_talon.slot_2.closedLoopPeakOutput = " << config.slot_2.closedLoopPeakOutput << std::endl;
	std::cout << "_talon.slot_2.closedLoopPeriod = " << config.slot_2.closedLoopPeriod << std::endl;
	std::cout << "_talon.slot_3.kP = " << config.slot_3.kP << std::endl;
	std::cout << "_talon.slot_3.kI = " << config.slot_3.kI << std::endl;
	std::cout << "_talon.slot_3.kD = " << config.slot_3.kD << std::endl;
	std::cout << "_talon.slot_3.kF = " << config.slot_3.kF << std::endl;
	std::cout << "_talon.slot_3.integralZone = " << config.slot_3.integralZone << std::endl;
	std::cout << "_talon.slot_3.allowableClosedloopError = " << config.slot_3.allowableClosedloopError << std::endl;
	std::cout << "_talon.slot_3.maxIntegralAccumulator = " << config.slot_3.maxIntegralAccumulator << std::endl;
	std::cout << "_talon.slot_3.closedLoopPeakOutput = " << config.slot_3.closedLoopPeakOutput << std::endl;
	std::cout << "_talon.slot_3.closedLoopPeriod = " << config.slot_3.closedLoopPeriod << std::endl;
	std::cout << "_talon.auxPIDPolarity = " << config.auxPIDPolarity << std::endl;
	std::cout << "_talon.motionCruiseVelocity = " << config.motionCruiseVelocity << std::endl;
	std::cout << "_talon.motionAcceleration = " << config.motionAcceleration << std::endl;
	std::cout << "_talon.motionProfileTrajectoryPeriod = " << config.motionProfileTrajectoryPeriod << std::endl;
	std::cout << "_talon.primaryPID.selectedFeedbackCoefficient = " << config.primaryPID.selectedFeedbackCoefficient << std::endl;
	std::cout << "_talon.primaryPID.sensorTerm = " << config.primaryPID.sensorTerm << std::endl;
	std::cout << "_talon.primaryPID.remoteSensorDeviceID = " << config.primaryPID.remoteSensorDeviceID << std::endl;
	std::cout << "_talon.primaryPID.remoteSensorSource = " << config.primaryPID.remoteSensorSource << std::endl;
	std::cout << "_talon.primaryPID.selectedFeedbackSensor = " << config.primaryPID.selectedFeedbackSensor << std::endl;
	std::cout << "_talon.auxilaryPID.selectedFeedbackCoefficient = " << config.auxilaryPID.selectedFeedbackCoefficient << std::endl;
	std::cout << "_talon.auxilaryPID.sensorTerm = " << config.auxilaryPID.sensorTerm << std::endl;
	std::cout << "_talon.auxilaryPID.remoteSensorDeviceID = " << config.auxilaryPID.remoteSensorDeviceID << std::endl;
	std::cout << "_talon.auxilaryPID.remoteSensorSource = " << config.auxilaryPID.remoteSensorSource << std::endl;
	std::cout << "_talon.auxilaryPID.selectedFeedbackSensor = " << config.auxilaryPID.selectedFeedbackSensor << std::endl;
	std::cout << "_talon.forwardLimitSwitchSource = " << config.forwardLimitSwitchSource << std::endl;
	std::cout << "_talon.reverseLimitSwitchSource = " << config.reverseLimitSwitchSource << std::endl;
	std::cout << "_talon.peakCurrentLimit = " << config.peakCurrentLimit << std::endl;
	std::cout << "_talon.peakCurrentDuration = " << config.peakCurrentDuration << std::endl;
	std::cout << "_talon.continuousCurrentLimit = " << config.continuousCurrentLimit << std::endl;
}
void PrintVictorConfigPart1(VictorSPXConfiguration &config) {
	std::cout << "_victor.openloopRamp = " << config.openloopRamp <<std::endl;
	std::cout << "_victor.closedloopRamp = " << config.closedloopRamp << std::endl;
	std::cout << "_victor.peakOutputForward = " << config.peakOutputForward << std::endl;
	std::cout << "_victor.peakOutputReverse = " << config.peakOutputReverse << std::endl;
	std::cout << "_victor.nominalOutputForward = " << config.nominalOutputForward << std::endl;
	std::cout << "_victor.nominalOutputReverse = " << config.nominalOutputReverse << std::endl;
	std::cout << "_victor.neutralDeadband = " << config.neutralDeadband << std::endl;
	std::cout << "_victor.voltageCompSaturation = " << config.voltageCompSaturation << std::endl;
	std::cout << "_victor.voltageMeasurementFilter = " << config.voltageMeasurementFilter << std::endl;
	std::cout << "_victor.velocityMeasurementPeriod = " << config.velocityMeasurementPeriod << std::endl;
	std::cout << "_victor.velocityMeasurementWindow = " << config.velocityMeasurementWindow << std::endl;
	std::cout << "_victor.forwardLimitSwitchDeviceID = " << config.forwardLimitSwitchDeviceID << std::endl;
	std::cout << "_victor.reverseLimitSwitchDeviceID = " << config.reverseLimitSwitchDeviceID << std::endl;
	std::cout << "_victor.forwardLimitSwitchNormal = " << config.forwardLimitSwitchNormal << std::endl;
	std::cout << "_victor.reverseLimitSwitchNormal = " << config.reverseLimitSwitchNormal << std::endl;
	std::cout << "_victor.forwardSoftLimitThreshold = " << config.forwardSoftLimitThreshold << std::endl;
	std::cout << "_victor.reverseSoftLimitThreshold = " << config.reverseSoftLimitThreshold << std::endl;
	std::cout << "_victor.forwardSoftLimitEnable = " << config.forwardSoftLimitEnable << std::endl;
	std::cout << "_victor.reverseSoftLimitEnable = " << config.reverseSoftLimitEnable << std::endl;
}
void PrintVictorConfigPart2(VictorSPXConfiguration &config) {
	std::cout << "_victor.slot_0.kP = " << config.slot_0.kP << std::endl;
	std::cout << "_victor.slot_0.kI = " << config.slot_0.kI << std::endl;
	std::cout << "_victor.slot_0.kD = " << config.slot_0.kD << std::endl;
	std::cout << "_victor.slot_0.kF = " << config.slot_0.kF << std::endl;
	std::cout << "_victor.slot_0.integralZone = " << config.slot_0.integralZone << std::endl;
	std::cout << "_victor.slot_0.allowableClosedloopError = " << config.slot_0.allowableClosedloopError << std::endl;
	std::cout << "_victor.slot_0.maxIntegralAccumulator = " << config.slot_0.maxIntegralAccumulator << std::endl;
	std::cout << "_victor.slot_0.closedLoopPeakOutput = " << config.slot_0.closedLoopPeakOutput << std::endl;
	std::cout << "_victor.slot_0.closedLoopPeriod = " << config.slot_0.closedLoopPeriod << std::endl;
	std::cout << "_victor.slot_1.kP = " << config.slot_1.kP << std::endl;
	std::cout << "_victor.slot_1.kI = " << config.slot_1.kI << std::endl;
	std::cout << "_victor.slot_1.kD = " << config.slot_1.kD << std::endl;
	std::cout << "_victor.slot_1.kF = " << config.slot_1.kF << std::endl;
	std::cout << "_victor.slot_1.integralZone = " << config.slot_1.integralZone << std::endl;
	std::cout << "_victor.slot_1.allowableClosedloopError = " << config.slot_1.allowableClosedloopError << std::endl;
	std::cout << "_victor.slot_1.maxIntegralAccumulator = " << config.slot_1.maxIntegralAccumulator << std::endl;
	std::cout << "_victor.slot_1.closedLoopPeakOutput = " << config.slot_1.closedLoopPeakOutput << std::endl;
	std::cout << "_victor.slot_1.closedLoopPeriod = " << config.slot_1.closedLoopPeriod << std::endl;
	std::cout << "_victor.slot_2.kP = " << config.slot_2.kP << std::endl;
	std::cout << "_victor.slot_2.kI = " << config.slot_2.kI << std::endl;
	std::cout << "_victor.slot_2.kD = " << config.slot_2.kD << std::endl;
	std::cout << "_victor.slot_2.kF = " << config.slot_2.kF << std::endl;
	std::cout << "_victor.slot_2.integralZone = " << config.slot_2.integralZone << std::endl;
	std::cout << "_victor.slot_2.allowableClosedloopError = " << config.slot_2.allowableClosedloopError << std::endl;
	std::cout << "_victor.slot_2.maxIntegralAccumulator = " << config.slot_2.maxIntegralAccumulator << std::endl;
	std::cout << "_victor.slot_2.closedLoopPeakOutput = " << config.slot_2.closedLoopPeakOutput << std::endl;
	std::cout << "_victor.slot_2.closedLoopPeriod = " << config.slot_2.closedLoopPeriod << std::endl;
	std::cout << "_victor.slot_3.kP = " << config.slot_3.kP << std::endl;
	std::cout << "_victor.slot_3.kI = " << config.slot_3.kI << std::endl;
	std::cout << "_victor.slot_3.kD = " << config.slot_3.kD << std::endl;
	std::cout << "_victor.slot_3.kF = " << config.slot_3.kF << std::endl;
	std::cout << "_victor.slot_3.integralZone = " << config.slot_3.integralZone << std::endl;
	std::cout << "_victor.slot_3.allowableClosedloopError = " << config.slot_3.allowableClosedloopError << std::endl;
	std::cout << "_victor.slot_3.maxIntegralAccumulator = " << config.slot_3.maxIntegralAccumulator << std::endl;
	std::cout << "_victor.slot_3.closedLoopPeakOutput = " << config.slot_3.closedLoopPeakOutput << std::endl;
	std::cout << "_victor.slot_3.closedLoopPeriod = " << config.slot_3.closedLoopPeriod << std::endl;
	std::cout << "_victor.auxPIDPolarity = " << config.auxPIDPolarity << std::endl;
	std::cout << "_victor.motionCruiseVelocity = " << config.motionCruiseVelocity << std::endl;
	std::cout << "_victor.motionAcceleration = " << config.motionAcceleration << std::endl;
	std::cout << "_victor.motionProfileTrajectoryPeriod = " << config.motionProfileTrajectoryPeriod << std::endl;
	std::cout << "_victor.primaryPID.selectedFeedbackCoefficient = " << config.primaryPID.selectedFeedbackCoefficient << std::endl;
	std::cout << "_victor.primaryPID.sensorTerm = " << (int) config.primaryPID.sensorTerm << std::endl;
	std::cout << "_victor.primaryPID.remoteSensorDeviceID = " << config.primaryPID.remoteSensorDeviceID << std::endl;
	std::cout << "_victor.primaryPID.remoteSensorSource = " << config.primaryPID.remoteSensorSource << std::endl;
	std::cout << "_victor.primaryPID.selectedFeedbackSensor = " << config.primaryPID.selectedFeedbackSensor << std::endl;
	std::cout << "_victor.auxilaryPID.selectedFeedbackCoefficient = " << config.auxilaryPID.selectedFeedbackCoefficient << std::endl;
	std::cout << "_victor.auxilaryPID.sensorTerm = " << (int) config.auxilaryPID.sensorTerm << std::endl;
	std::cout << "_victor.auxilaryPID.remoteSensorDeviceID = " << config.auxilaryPID.remoteSensorDeviceID << std::endl;
	std::cout << "_victor.auxilaryPID.remoteSensorSource = " << config.auxilaryPID.remoteSensorSource << std::endl;
	std::cout << "_victor.auxilaryPID.selectedFeedbackSensor = " << config.auxilaryPID.selectedFeedbackSensor << std::endl;

	std::cout << "_victor.forwardLimitSwitchSource = " << config.forwardLimitSwitchSource << std::endl;
	std::cout << "_victor.reverseLimitSwitchSource = " << config.reverseLimitSwitchSource << std::endl;


}




#endif /* SRC_PRINTFUNCTIONS_H_ */
