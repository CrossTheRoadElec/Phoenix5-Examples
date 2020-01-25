/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/TimedRobot.h>
#include "ctre/Phoenix.h"
#include "frc/Joystick.h"
#include <vector>
#include <string>

#define INSTRUMENT_COUNT 5

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;

  void TestInit() override;
  void TestPeriodic() override;

  Orchestra *_orchestra;
  frc::Joystick *_joy;
  TalonFX *_instruments[INSTRUMENT_COUNT];
  std::vector<std::string> _songs;

  
  int _songSelection = 0;
  int _lastButton = 0;
  int _lastPOV = 0;
};
