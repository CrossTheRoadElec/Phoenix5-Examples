// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.ctre.phoenix.sensors.Pigeon2_Faults;

public class Pigeon2Subsystem extends PigeonSubsystem {
  private static WPI_Pigeon2 m_pigeon2;
  private static Pigeon2_Faults m_pigeonFaults = new Pigeon2_Faults();
  /** Creates a new ExampleSubsystem. */
  private Pigeon2Subsystem(WPI_Pigeon2 device) {
    super(device);
    m_pigeon2 = device;
  }
  public Pigeon2Subsystem(int deviceId) {
    this(new WPI_Pigeon2(deviceId));
  }
  public Pigeon2Subsystem(int deviceId, String canbus) {
    this(new WPI_Pigeon2(deviceId, canbus));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_pigeon2.getFaults(m_pigeonFaults);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public Pigeon2_Faults getFaults() { return m_pigeonFaults; }
  public boolean getFault() { return m_pigeonFaults.hasAnyFault(); }

  public String getFaultMessage() { 
    if(!m_pigeonFaults.hasAnyFault()) return "No faults";
    String retval = "";
    retval += m_pigeonFaults.APIError ? "APIError, " : "";
    retval += m_pigeonFaults.AccelFault ? "AccelFault, " : "";
    retval += m_pigeonFaults.BootIntoMotion ? "BootIntoMotion, " : "";
    retval += m_pigeonFaults.DataAcquiredLate ? "DataAcquiredLate, " : "";
    retval += m_pigeonFaults.GyroFault ? "GyroFault, " : "";
    retval += m_pigeonFaults.HardwareFault ? "HardwareFault, " : "";
    retval += m_pigeonFaults.MagnetometerFault ? "MagnetometerFault, " : "";
    retval += m_pigeonFaults.MotionDriverTookTooLong ? "MotionDriverTookTooLong, " : "";
    retval += m_pigeonFaults.ResetDuringEn ? "ResetDuringEn, " : "";
    retval += m_pigeonFaults.SaturatedAccel ? "SaturatedAccel, " : "";
    retval += m_pigeonFaults.SaturatedMag ? "SaturatedMag, " : "";
    retval += m_pigeonFaults.SaturatedRotVelocity ? "SaturatedRotVelocity, " : "";
    return retval;
  }
}
