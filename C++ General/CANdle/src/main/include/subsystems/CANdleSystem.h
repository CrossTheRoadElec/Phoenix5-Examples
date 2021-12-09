// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "ctre/phoenix/led/CANdle.h"
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/InstantCommand.h>
#include "Constants.h"

class CANdleSystem : public frc2::SubsystemBase {
    ctre::phoenix::led::CANdle m_candle {candleID, ""};
    int LedCount = 300;

    ctre::phoenix::led::Animation *m_toAnimate = NULL;

    int m_r, m_g, m_b;
    double m_modulatedOut;

    enum AnimationTypes {
        ColorFlow,
        Fire,
        Larson,
        Rainbow,
        RgbFade,
        SingleFade,
        Strobe,
        Twinkle,
        TwinkleOff,
        SetAll
    };
    AnimationTypes m_currentAnimation;

 public:

    CANdleSystem();
    ~CANdleSystem();

    void IncrementAnimation();
    void DecrementAnimation();
    void SetColors();

    /* Wrappers so we can access the CANdle from the subsystem */
    double GetVbat();
    double Get5V();
    double GetCurrent();
    double GetTemperature();
    double GetModulatedOutput();
    void ConfigBrightness(double percent);
    void ConfigLos(bool disableWhenLos);
    void ConfigLedType(ctre::phoenix::led::LEDStripType type);
    void ConfigStatusLedBehavior(bool offWhenActive);

    void ChangeAnimation(AnimationTypes toChange);

    void UpdateSetLed(std::function<double()> r, std::function<double()> g, std::function<double()> b, std::function<double()> modulatedVbat);

    void Periodic();

    void SimulationPeriodic();
};
