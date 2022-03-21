/**
 * Phoenix Software License Agreement
 *
 * Copyright (C) Cross The Road Electronics.  All rights
 * reserved.
 * 
 * Cross The Road Electronics (CTRE) licenses to you the right to 
 * use, publish, and distribute copies of CRF (Cross The Road) firmware files (*.crf) and 
 * Phoenix Software API Libraries ONLY when in use with CTR Electronics hardware products
 * as well as the FRC roboRIO when in use in FRC Competition.
 * 
 * THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT
 * WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT
 * LIMITATION, ANY WARRANTY OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * CROSS THE ROAD ELECTRONICS BE LIABLE FOR ANY INCIDENTAL, SPECIAL, 
 * INDIRECT OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF
 * PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
 * BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE
 * THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER
 * SIMILAR COSTS, WHETHER ASSERTED ON THE BASIS OF CONTRACT, TORT
 * (INCLUDING NEGLIGENCE), BREACH OF WARRANTY, OR OTHERWISE
 */

#pragma once

#include "ctre/phoenix/led/CANdle.h"
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/InstantCommand.h>
#include "Constants.h"

class CANdleSystem : public frc2::SubsystemBase {
    ctre::phoenix::led::CANdle m_candle {candleID, "1"};
    int LEDS_PER_ANIMATION = 30;
    int m_candleChannel = 0;
    bool m_clearAllAnims = false;
    bool m_last5V = false;
    bool m_animDirection = false;
    bool m_setAnim = false;

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
    void Toggle5VOverride();
    void ToggleAnimDirection();
    int GetMaximumAnimationCount();
    void ClearAllAnims();

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
