// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ReportingCommand;
import frc.robot.subsystems.Pigeon1Subsystem;
import frc.robot.subsystems.Pigeon2Subsystem;
import frc.robot.subsystems.ReportingSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Pigeon2Subsystem m_pigeon2subsystem = new Pigeon2Subsystem(Constants.Pigeon2ID, "rio");
  private final Pigeon1Subsystem m_pigeon1subsystem = new Pigeon1Subsystem(new TalonSRX(Constants.Pigeon1ID));
  private final ReportingSubsystem m_reportingSubsystem = new ReportingSubsystem();

  private final XboxController m_joy = new XboxController(0);

  private final ReportingCommand m_reportingCommand = new ReportingCommand(m_reportingSubsystem, m_pigeon2subsystem, m_pigeon1subsystem);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_reportingSubsystem.setDefaultCommand(m_reportingCommand);
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
    new JoystickButton(m_joy, Constants.ZeroPigeonYaws).whenPressed(
        ()->{
          m_pigeon2subsystem.setYaw(0); 
          m_pigeon1subsystem.setYaw(0);
          m_pigeon2subsystem.setAccumZ(0);
          m_pigeon1subsystem.setAccumZ(0);
        });
    new JoystickButton(m_joy, Constants.AddPigeonYaws).whenPressed(
      ()->{
        m_pigeon2subsystem.addYaw(10);
        m_pigeon1subsystem.addYaw(10);
        m_pigeon2subsystem.setAccumZ(5);
        m_pigeon1subsystem.setAccumZ(5);
      }
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_reportingCommand;
  }
}
