
#pragma once

#include <frc/TimedRobot.h>
#include <frc/Joystick.h>
#include <ctre/Phoenix.h>

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;

  void TestInit() override;
  void TestPeriodic() override;

private:
  WPI_CANCoder _CANCoder{0, "rio"}; // Rename "rio" to match the CANivore device name if using a CANivore
  frc::Joystick _joy{0};
};
