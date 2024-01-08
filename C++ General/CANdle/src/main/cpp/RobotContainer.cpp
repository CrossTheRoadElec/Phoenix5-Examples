// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "Constants.h"
#include "commands/CANdlePrintCommands.h"
#include "commands/CANdleConfigCommands.h"
#include <frc/XboxController.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/POVButton.h>
#include <frc2/command/RunCommand.h>

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
  m_subsystem.SetDefaultCommand(frc2::RunCommand([this] {
      m_subsystem.UpdateSetLed([this]{return m_joy.GetLeftTriggerAxis();},
                              [this]{return m_joy.GetRightTriggerAxis();},
                              [this]{return m_joy.GetLeftX();},
                              [this]{return m_joy.GetRightY();});
    },
    {&m_subsystem}
  ));

  // Configure the button bindings
  ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here

  frc2::JoystickButton(&m_joy, nextAnimID).When([this]{m_subsystem.IncrementAnimation();});
  frc2::JoystickButton(&m_joy, prevAnimID).OnTrue([this]{m_subsystem.DecrementAnimation();});
  frc2::JoystickButton(&m_joy, setLedID).OnTrue([this]{m_subsystem.SetColors();});

  frc2::JoystickButton(&m_joy, printBusVID).OnTrue(CANdlePrintCommands::PrintVBat(&m_subsystem));
  frc2::JoystickButton(&m_joy, print5VID).OnTrue(CANdlePrintCommands::Print5V(&m_subsystem));
  frc2::JoystickButton(&m_joy, printCurrentID).OnTrue(CANdlePrintCommands::PrintCurrent(&m_subsystem));
  frc2::JoystickButton(&m_joy, printTemperatureID).OnTrue(CANdlePrintCommands::PrintTemperature(&m_subsystem));
  frc2::JoystickButton(&m_joy, printModulatedOutput).OnTrue(CANdlePrintCommands::PrintModulatedOutput(&m_subsystem));

  frc2::POVButton(&m_joy, configMaxBrightness).OnTrue(CANdleConfigCommands::ConfigBrightness(&m_subsystem, 1));
  frc2::POVButton(&m_joy, configMidBrightness).OnTrue(CANdleConfigCommands::ConfigBrightness(&m_subsystem, 0.5));
  frc2::POVButton(&m_joy, configOffBrigthness).OnTrue(CANdleConfigCommands::ConfigBrightness(&m_subsystem, 0));
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return NULL;
}
