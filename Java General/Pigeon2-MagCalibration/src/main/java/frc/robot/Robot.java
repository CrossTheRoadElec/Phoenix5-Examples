
package frc.robot;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  Pigeon2 m_toCalibrate = new Pigeon2(1, "rio");
  Joystick m_joy = new Joystick(0);
  @Override
  public void robotInit() {}

  @Override
  public void robotPeriodic() {
    if(m_joy.getRawButtonPressed(1)) {
      System.out.println("Beginning Mag Cal");
      /* Begin calibration */
      m_toCalibrate.configSetParameter(ParamEnum.eMagCalRunning, 1, 0, 0);
    }
    if(m_joy.getRawButtonPressed(2)) {
      System.out.println("Stopping and applying Mag Cal");
      /* Apply Calibration */
      m_toCalibrate.configSetParameter(ParamEnum.eMagCal, 0, 0, 0);
      /* Stop calibration */
      m_toCalibrate.configSetParameter(ParamEnum.eMagCalRunning, 0, 0, 0);
    }
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    /* Report the status of the calibration in periodic */
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
