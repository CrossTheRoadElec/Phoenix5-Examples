#pragma once

#include "ctre/Phoenix.h"

/* FollowerProfileConfiguration inherits TalonFXConfiguration so it has all the default configs + the unique configs for the follower talon */
class FollowerProfileConfiguration : public TalonFXConfiguration
{
public:
    FollowerProfileConfiguration() : TalonFXConfiguration()
    {
        primaryPID.selectedFeedbackSensor = FeedbackDevice::IntegratedSensor;
        neutralDeadband = 0.001; /* 0.1% super small for best low-speed control */
    }
};