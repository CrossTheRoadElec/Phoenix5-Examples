#pragma once

#include "ctrlib/Drive/IDrivetrain.h"
#include "ctrlib/Drive/Styles.h"
#include "ctrlib/PigeonImu.h"
#include "ctrlib/Utilities.h"
#include "ctrlib/Stopwatch.h"
#include "ctrlib/Motion/ServoParameters.h"

namespace CTRE {
namespace Motion {
class ServoHoldHeadingWithImu{
public:
	ServoParameters* _servoParams = new ServoParameters();
	ServoHoldHeadingWithImu(CTRE::PigeonImu *pigeonImu, CTRE::Drive::IDrivetrain *drivetrain, CTRE::Drive::Styles::Basic selectedStyle, ServoParameters *parameters, float targetHeading, float maxOutput);
	ServoHoldHeadingWithImu(CTRE::PigeonImu *pigeonImu, CTRE::Drive::IDrivetrain *drivetrain, CTRE::Drive::Styles::Basic selectedStyle);
	void Set(CTRE::Drive::Styles::Basic mode, float forward, float strafe);
	float GetImuHeading();
private:
    CTRE::PigeonImu *_pidgey;
    CTRE::Drive::IDrivetrain *_driveTrain;
    CTRE::Drive::Styles::Basic _selectedStyle;
    float _targetHeading = 0;
    float _maxOutput = 0;
    bool _isRunning = false;
    bool _isDone = false;
    unsigned char _state = 0;

    void GoStraight(float Y, float targetheading);
    bool _isCompensating = false;
    bool _enableCompensating = false;
    void Enable(bool state);
    void Disable();
};
}}
