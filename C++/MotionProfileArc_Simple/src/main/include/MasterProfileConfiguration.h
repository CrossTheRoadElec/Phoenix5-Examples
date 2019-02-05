#pragma once

#include "ctre/Phoenix.h"

class MasterProfileConfiguration : public TalonSRXConfiguration
{
public:
    MasterProfileConfiguration(TalonSRX *otherTalon, PigeonIMU *pigeon) : TalonSRXConfiguration()
    {
        primaryPID.selectedFeedbackSensor = FeedbackDevice::SensorSum;
        auxiliaryPID.selectedFeedbackSensor = FeedbackDevice::RemoteSensor1;
        neutralDeadband = 0.001; /* 0.1% super small for best low-speed control */

        slot0.kF = 0.35;
        slot0.kP = 0.8;
        slot0.kI = 0.0;
        slot0.kD = 80;
        slot0.integralZone = 400;
        slot0.closedLoopPeakOutput = 1.0;

        slot1.kF = 0;
        slot1.kP = 1.0;
        slot1.kI = 0.0;
        slot1.kD = 0.0;
        slot1.integralZone = 400;
        slot1.closedLoopPeakOutput = 0.5;

        remoteFilter0.remoteSensorSource = RemoteSensorSource::RemoteSensorSource_TalonSRX_SelectedSensor;
        remoteFilter0.remoteSensorDeviceID = otherTalon->GetDeviceID();

        remoteFilter1.remoteSensorSource = RemoteSensorSource::RemoteSensorSource_Pigeon_Yaw;
        remoteFilter1.remoteSensorDeviceID = pigeon->GetDeviceNumber();

        sum0Term = FeedbackDevice::QuadEncoder;
        sum1Term = FeedbackDevice::RemoteSensor0;

        auxPIDPolarity = false;
    }
};