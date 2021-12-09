#pragma once

#include <frc/TimedRobot.h>
#include <frc/Joystick.h>
#include <frc/drive/DifferentialDrive.h>
#include "ctre/Phoenix.h"

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;

  void SimulationInit() override;
  void SimulationPeriodic() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;

  void TestInit() override;
  void TestPeriodic() override;

 private:
  /* Master Talons for arcade drive */
  WPI_TalonSRX _left{1};
  WPI_TalonSRX _rght{2};

  /* Construct drivetrain by providing master motor controllers */
  frc::DifferentialDrive _drive{_left, _rght};
  frc::Joystick _joy{0};  /* Joystick for control */

  int _loops = 0; // slow print to the DS
};
