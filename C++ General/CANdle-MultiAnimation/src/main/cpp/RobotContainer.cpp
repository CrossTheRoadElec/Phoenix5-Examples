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
/**
 * Description:
 * The CANdle MultiAnimation example demonstrates using multiple animations with CANdle.
 * This example has the robot using a Command Based template to control the CANdle.
 * 
 * This example uses:
 * - A CANdle wired on the CAN Bus, with a 5m led strip attached for the extra animatinos.
 * 
 * Controls (with Xbox controller):
 * Right Bumper: Increment animation
 * Left Bumper: Decrement animation
 * Start Button: Switch to setting the first 8 LEDs a unique combination of colors
 * POV Right: Configure maximum brightness for the CANdle
 * POV Down: Configure medium brightness for the CANdle
 * POV Left: Configure brightness to 0 for the CANdle
 * POV Up: Change the direction of Rainbow and Fire, must re-select the animation to take affect
 * A: Print the VBat voltage in Volts
 * B: Print the 5V voltage in Volts
 * X: Print the current in amps
 * Y: Print the temperature in degrees C
 * 
 * This example uses a development build based on 5.21.2.
 * 
 * Supported Version:
 * 	- CANdle: 22.1.1.0
 */

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

  frc2::JoystickButton(&m_joy, nextAnimID).WhenPressed([this]{m_subsystem.IncrementAnimation();});
  frc2::JoystickButton(&m_joy, prevAnimID).WhenPressed([this]{m_subsystem.DecrementAnimation();});
  frc2::JoystickButton(&m_joy, setLedID).WhenPressed([this]{m_subsystem.SetColors();});
  
  frc2::JoystickButton(&m_joy, printBusVID).WhenPressed(CANdlePrintCommands::PrintVBat(&m_subsystem));
  frc2::JoystickButton(&m_joy, print5VID).WhenPressed(CANdlePrintCommands::Print5V(&m_subsystem));
  frc2::JoystickButton(&m_joy, printCurrentID).WhenPressed(CANdlePrintCommands::PrintCurrent(&m_subsystem));
  frc2::JoystickButton(&m_joy, printTemperatureID).WhenPressed(CANdlePrintCommands::PrintTemperature(&m_subsystem));
  frc2::JoystickButton(&m_joy, printModulatedOutput).WhenPressed(CANdlePrintCommands::PrintModulatedOutput(&m_subsystem));

  frc2::POVButton(&m_joy, configMaxBrightness).WhenPressed(CANdleConfigCommands::ConfigBrightness(&m_subsystem, 1));
  frc2::POVButton(&m_joy, configMidBrightness).WhenPressed(CANdleConfigCommands::ConfigBrightness(&m_subsystem, 0.5));
  frc2::POVButton(&m_joy, configOffBrigthness).WhenPressed(CANdleConfigCommands::ConfigBrightness(&m_subsystem, 0));
  frc2::POVButton(&m_joy, ChangeDirectionAngle).WhenPressed([this]{m_subsystem.ToggleAnimDirection();});

  frc2::JoystickButton(&m_joy, 9).WhenPressed([this]{m_subsystem.ClearAllAnims();});
  frc2::JoystickButton(&m_joy, 10).WhenPressed([this]{m_subsystem.Toggle5VOverride();});
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return NULL;
}
