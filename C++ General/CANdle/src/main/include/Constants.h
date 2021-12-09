// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "frc/XboxController.h"
/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

constexpr int controllerID = 0;
constexpr int candleID = 1;
constexpr int nextAnimID = frc::XboxController::Button::kRightBumper;
constexpr int prevAnimID = frc::XboxController::Button::kLeftBumper;
constexpr int setLedID = frc::XboxController::Button::kStart;

constexpr int printBusVID = frc::XboxController::Button::kA;
constexpr int print5VID = frc::XboxController::Button::kB;
constexpr int printCurrentID = frc::XboxController::Button::kX;
constexpr int printTemperatureID = frc::XboxController::Button::kY;
constexpr int printModulatedOutput = frc::XboxController::Button::kBack;

constexpr int configMaxBrightness = 90;
constexpr int configMidBrightness = 180;
constexpr int configOffBrigthness = 270;
