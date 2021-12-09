// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

public class Pigeon1Subsystem extends PigeonSubsystem {
  private WPI_PigeonIMU m_pigeon1;
  /** Creates a new ExampleSubsystem. */
  public Pigeon1Subsystem(WPI_PigeonIMU device) {
    super(device);
    m_pigeon1 = device;
  }
  public Pigeon1Subsystem(int deviceId) {
    this(new WPI_PigeonIMU(deviceId));
  }
  public Pigeon1Subsystem(TalonSRX srxDevice) {
    this(new WPI_PigeonIMU(srxDevice));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public double getYaw() { return m_pigeon1.getYaw(); }
  public double getPitch() { return m_pigeon1.getPitch(); }
  public double getRoll() { return m_pigeon1.getRoll(); }
  public void setYaw(double yaw) { m_pigeon1.setYaw(yaw); }
  public double getCompass() { return m_pigeon1.getCompassHeading(); }
  public boolean getFault() { return false; }

  public String getFaultMessage() { return "Pigeon1 has no faults"; }
}
