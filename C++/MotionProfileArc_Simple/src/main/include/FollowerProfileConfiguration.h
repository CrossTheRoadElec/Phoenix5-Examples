#pragma once

#include "ctre/Phoenix.h"

class FollowerProfileConfiguration : public TalonSRXConfiguration
{
public:
    FollowerProfileConfiguration() : TalonSRXConfiguration()
    {
        primaryPID.selectedFeedbackSensor = FeedbackDevice::QuadEncoder;
        neutralDeadband = 0.001; /* 0.1% super small for best low-speed control */
    }
};