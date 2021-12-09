// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
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
        case ColorFlow:
            m_toAnimate = new ColorFlowAnimation(128, 20, 70, 0, 0.7, LedCount, ColorFlowAnimation::Direction::Forward);
            break;
        case Fire:
            m_toAnimate = new FireAnimation(0.5, 0.7, LedCount, 0.7, 0.5);
            break;
        case Larson:
            m_toAnimate = new LarsonAnimation(0, 255, 46, 0, 1, LedCount, LarsonAnimation::BounceMode::Front, 3);
            break;
        case Rainbow:
            m_toAnimate = new RainbowAnimation(1, 0.1, LedCount);
            break;
        case RgbFade:
            m_toAnimate = new RgbFadeAnimation(0.7, 0.4, LedCount);
            break;
        case SingleFade:
            m_toAnimate = new SingleFadeAnimation(50, 2, 200, 0, 0.5, LedCount);
            break;
        case Strobe:
            m_toAnimate = new StrobeAnimation(240, 10, 180, 0, 98.0 / 256.0, LedCount);
            break;
        case Twinkle:
            m_toAnimate = new TwinkleAnimation(30, 70, 60, 0, 0.4, LedCount, TwinkleAnimation::TwinklePercent::Percent6);
            break;
        case TwinkleOff:
            m_toAnimate = new TwinkleOffAnimation(70, 90, 175, 0, 0.8, LedCount, TwinkleOffAnimation::TwinkleOffPercent::Percent100);
            break;
        default:
        case SetAll:
            m_toAnimate = NULL;
            break;
    }
    std::cout << "Changed to " << std::to_string(m_currentAnimation) << std::endl;
}

void CANdleSystem::Periodic() {
    // This method will be called once per scheduler run
    if(m_toAnimate == NULL) {
        m_candle.SetLEDs(m_r, m_g, m_b);
    } else {
        m_candle.Animate(*m_toAnimate);
    }
    m_candle.ModulateVBatOutput(m_modulatedOut);
}

void CANdleSystem::SimulationPeriodic() {
    // This method will be called once per scheduler run during simulation
}
