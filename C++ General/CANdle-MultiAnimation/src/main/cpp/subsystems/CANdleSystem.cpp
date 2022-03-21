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
#include <iostream>
#include "subsystems/CANdleSystem.h"
#include "ctre/phoenix/led/ColorFlowAnimation.h"
#include "ctre/phoenix/led/FireAnimation.h"
#include "ctre/phoenix/led/LarsonAnimation.h"
#include "ctre/phoenix/led/RainbowAnimation.h"
#include "ctre/phoenix/led/RgbFadeAnimation.h"
#include "ctre/phoenix/led/SingleFadeAnimation.h"
#include "ctre/phoenix/led/StrobeAnimation.h"
#include "ctre/phoenix/led/TwinkleAnimation.h"
#include "ctre/phoenix/led/TwinkleOffAnimation.h"

using namespace ctre::phoenix::led;

CANdleSystem::CANdleSystem() {
    ChangeAnimation(AnimationTypes::SetAll);
    CANdleConfiguration configAll {};
    configAll.statusLedOffWhenActive = true;
    configAll.disableWhenLOS = false;
    configAll.stripType = LEDStripType::GRB;
    configAll.brightnessScalar = 0.1;
    configAll.vBatOutputMode = VBatOutputMode::Modulated;
    m_candle.ConfigAllSettings(configAll, 100);
}

CANdleSystem::~CANdleSystem() {
    if(m_toAnimate != NULL) delete m_toAnimate;
}

void CANdleSystem::IncrementAnimation() {
    switch(m_currentAnimation) {
        case ColorFlow: ChangeAnimation(AnimationTypes::Fire); break;
        case Fire: ChangeAnimation(AnimationTypes::Larson); break;
        case Larson: ChangeAnimation(AnimationTypes::Rainbow); break;
        case Rainbow: ChangeAnimation(AnimationTypes::RgbFade); break;
        case RgbFade: ChangeAnimation(AnimationTypes::SingleFade); break;
        case SingleFade: ChangeAnimation(AnimationTypes::Strobe); break;
        case Strobe: ChangeAnimation(AnimationTypes::Twinkle); break;
        case Twinkle: ChangeAnimation(AnimationTypes::TwinkleOff); break;
        case TwinkleOff: ChangeAnimation(AnimationTypes::ColorFlow); break;
        case SetAll: ChangeAnimation(AnimationTypes::ColorFlow); break;
    }
}
void CANdleSystem::DecrementAnimation() {
    switch(m_currentAnimation) {
        case ColorFlow: ChangeAnimation(AnimationTypes::TwinkleOff); break;
        case Fire: ChangeAnimation(AnimationTypes::ColorFlow); break;
        case Larson: ChangeAnimation(AnimationTypes::Fire); break;
        case Rainbow: ChangeAnimation(AnimationTypes::Larson); break;
        case RgbFade: ChangeAnimation(AnimationTypes::Rainbow); break;
        case SingleFade: ChangeAnimation(AnimationTypes::RgbFade); break;
        case Strobe: ChangeAnimation(AnimationTypes::SingleFade); break;
        case Twinkle: ChangeAnimation(AnimationTypes::Strobe); break;
        case TwinkleOff: ChangeAnimation(AnimationTypes::Twinkle); break;
        case SetAll: ChangeAnimation(AnimationTypes::ColorFlow); break;
    }
}
void CANdleSystem::SetColors() {
    ChangeAnimation(AnimationTypes::SetAll);
}
void CANdleSystem::Toggle5VOverride() {
    m_candle.configV5Enabled(m_last5V);
    m_last5V = !m_last5V;
}
void CANdleSystem::ToggleAnimDirection() {
    m_animDirection = !m_animDirection;
}
int CANdleSystem::GetMaximumAnimationCount() {
    return m_candle.GetMaxSimultaneousAnimationCount();
}


void CANdleSystem::UpdateSetLed(std::function<double()> r, std::function<double()> g, std::function<double()> b, std::function<double()> modulatedVbat) {
    m_r = r() * 255;
    m_g = g() * 255;
    m_b = b() * 255;
    m_modulatedOut = modulatedVbat();
}

/* Wrappers so we can access the CANdle from the subsystem */
double CANdleSystem::GetVbat() { return m_candle.GetBusVoltage(); }
double CANdleSystem::Get5V() { return m_candle.Get5VRailVoltage(); }
double CANdleSystem::GetCurrent() { return m_candle.GetCurrent(); }
double CANdleSystem::GetTemperature() { return m_candle.GetTemperature(); }
double CANdleSystem::GetModulatedOutput() { return m_candle.GetVBatModulation(); }
void CANdleSystem::ConfigBrightness(double percent) { m_candle.ConfigBrightnessScalar(percent, 0); }
void CANdleSystem::ConfigLos(bool disableWhenLos) { m_candle.ConfigLOSBehavior(disableWhenLos, 0); }
void CANdleSystem::ConfigLedType(LEDStripType type) { m_candle.ConfigLEDType(type, 0); }
void CANdleSystem::ConfigStatusLedBehavior(bool offWhenActive) { m_candle.ConfigStatusLedState(offWhenActive, 0); }

void CANdleSystem::ChangeAnimation(AnimationTypes toChange) {
    m_currentAnimation = toChange;
    
    if(m_toAnimate != NULL) delete m_toAnimate;

    switch(toChange)
    {
            default:
            case ColorFlow:
                m_candleChannel = 0;
                m_toAnimate = new ColorFlowAnimation(128, 20, 70, 0, 0.7, LEDS_PER_ANIMATION, ColorFlowAnimation::Direction::Forward, m_candleChannel * LEDS_PER_ANIMATION + 8);
                break;
            case Fire:
                m_candleChannel = 1;
                m_toAnimate = new FireAnimation(0.5, 0.7, LEDS_PER_ANIMATION, 0.8, 0.5, m_animDirection, m_candleChannel * LEDS_PER_ANIMATION + 8);
                break;
            case Larson:
                m_candleChannel = 2;
                m_toAnimate = new LarsonAnimation(0, 255, 46, 0, 0.1, LEDS_PER_ANIMATION, LarsonAnimation::BounceMode::Front, 3, m_candleChannel * LEDS_PER_ANIMATION + 8);
                break;
            case Rainbow:
                m_candleChannel = 3;
                m_toAnimate = new RainbowAnimation(1, 0.7, LEDS_PER_ANIMATION, m_animDirection, m_candleChannel * LEDS_PER_ANIMATION + 8);
                break;
            case RgbFade:
                m_candleChannel = 4;
                m_toAnimate = new RgbFadeAnimation(0.7, 0.4, LEDS_PER_ANIMATION, m_candleChannel * LEDS_PER_ANIMATION + 8);
                break;
            case SingleFade:
                m_candleChannel = 5;
                m_toAnimate = new SingleFadeAnimation(50, 2, 200, 0, 0.5, LEDS_PER_ANIMATION, m_candleChannel * LEDS_PER_ANIMATION + 8);
                break;
            case Strobe:
                m_candleChannel = 6;
                m_toAnimate = new StrobeAnimation(240, 10, 180, 0, 0.01, LEDS_PER_ANIMATION, m_candleChannel * LEDS_PER_ANIMATION + 8);
                break;
            case Twinkle:
                m_candleChannel = 7;
                m_toAnimate = new TwinkleAnimation(30, 70, 60, 0, 0.4, LEDS_PER_ANIMATION, TwinkleAnimation::TwinklePercent::Percent42, m_candleChannel * LEDS_PER_ANIMATION + 8);
                break;
            case TwinkleOff:
                m_candleChannel = 8;
                m_toAnimate = new TwinkleOffAnimation(70, 90, 175, 0, 0.2, LEDS_PER_ANIMATION, TwinkleOffAnimation::TwinkleOffPercent::Percent76, m_candleChannel * LEDS_PER_ANIMATION + 8);
                break;

            case SetAll:
                m_toAnimate = NULL;
                break;
    }
    std::cout << "Changed to " << std::to_string(m_currentAnimation) << std::endl;
}
void CANdleSystem::ClearAllAnims() {m_clearAllAnims = true;}

void CANdleSystem::Periodic() {
    // This method will be called once per scheduler run
    if(m_toAnimate == NULL) {
        if(!m_setAnim) {
            /* Only setLEDs once, because every set will transmit a frame */
            m_candle.SetLEDs(255, 255, 255, 0, 0, 1);
            m_candle.SetLEDs(255, 255, 0, 0, 1, 1);
            m_candle.SetLEDs(255, 0, 255, 0, 2, 1);
            m_candle.SetLEDs(255, 0, 0, 0, 3, 1);
            m_candle.SetLEDs(0, 255, 255, 0, 4, 1);
            m_candle.SetLEDs(0, 255, 0, 0, 5, 1);
            m_candle.SetLEDs(0, 0, 0, 0, 6, 1);
            m_candle.SetLEDs(0, 0, 255, 0, 7, 1);
            m_setAnim = true;
        }
    } else {
        m_candle.Animate(*m_toAnimate, m_candleChannel);
        m_setAnim = false;
    }

    if(m_clearAllAnims) {
        m_clearAllAnims = false;
        for(int i = 0; i < 10; ++i) {
            m_candle.ClearAnimation(i); // If using the 5.21.2 build of Phoenix, comment this out as there is a bug that causes a compilation error
                                        // This bug has been fixed in dev build 5.21.2-12-gfdb3388 and any full release after 5.21.2
        }
    }
}

void CANdleSystem::SimulationPeriodic() {
    // This method will be called once per scheduler run during simulation
}
