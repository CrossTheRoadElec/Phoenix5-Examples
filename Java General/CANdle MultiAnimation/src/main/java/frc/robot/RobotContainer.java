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
 * Supported Version:
 * 	- CANdle: 22.1.1.0
 */

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.CANdleSystem;
import frc.robot.commands.CANdleConfigCommands;
import frc.robot.commands.CANdlePrintCommands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final XboxController joy = new XboxController(Constants.JoystickId);
  
  private final CANdleSystem m_candleSubsystem = new CANdleSystem(joy);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(joy, Constants.BlockButton).whenPressed(m_candleSubsystem::setColors, m_candleSubsystem);
    new JoystickButton(joy, Constants.IncrementAnimButton).whenPressed(m_candleSubsystem::incrementAnimation, m_candleSubsystem);
    new JoystickButton(joy, Constants.DecrementAnimButton).whenPressed(m_candleSubsystem::decrementAnimation, m_candleSubsystem);

    new POVButton(joy, Constants.MaxBrightnessAngle).whenPressed(new CANdleConfigCommands.ConfigBrightness(m_candleSubsystem, 1.0));
    new POVButton(joy, Constants.MidBrightnessAngle).whenPressed(new CANdleConfigCommands.ConfigBrightness(m_candleSubsystem, 0.3));
    new POVButton(joy, Constants.ZeroBrightnessAngle).whenPressed(new CANdleConfigCommands.ConfigBrightness(m_candleSubsystem, 0));
    new POVButton(joy, Constants.ChangeDirectionAngle).whenPressed(()->m_candleSubsystem.toggleAnimDirection(), m_candleSubsystem);

    new JoystickButton(joy, 9).whenPressed(()->m_candleSubsystem.clearAllAnims(), m_candleSubsystem);
    new JoystickButton(joy, 10).whenPressed(()->m_candleSubsystem.toggle5VOverride(), m_candleSubsystem);

    new JoystickButton(joy, Constants.VbatButton).whenPressed(new CANdlePrintCommands.PrintVBat(m_candleSubsystem));
    new JoystickButton(joy, Constants.V5Button).whenPressed(new CANdlePrintCommands.Print5V(m_candleSubsystem));
    new JoystickButton(joy, Constants.CurrentButton).whenPressed(new CANdlePrintCommands.PrintCurrent(m_candleSubsystem));
    new JoystickButton(joy, Constants.TemperatureButton).whenPressed(new CANdlePrintCommands.PrintTemperature(m_candleSubsystem));
  }
}
