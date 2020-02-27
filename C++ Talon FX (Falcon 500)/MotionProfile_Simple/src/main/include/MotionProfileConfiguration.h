#pragma once

#include "ctre/Phoenix.h"

/* Class that inherits TalonSRXConfiguration so it has all the default configs + modified the configs it cares about */
class MotionProfileConfiguration : public TalonFXConfiguration
{
public:
    MotionProfileConfiguration() : TalonFXConfiguration()
    {
        primaryPID.selectedFeedbackSensor = FeedbackDevice::IntegratedSensor;
        neutralDeadband = 0.001; /* 0.1% super small for best low-speed control */

        slot0.kF = 1023.0 / 68000.0;
        slot0.kP = 1.0;
        slot0.kI = 0.0;
        slot0.kD = 0.0;
        slot0.integralZone = 400;
        slot0.closedLoopPeakOutput = 1.0;
    }
};