package frc.robot.subsystems;

import com.ctre.phoenix.sensors.BasePigeon;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class PigeonSubsystem extends SubsystemBase {

    private BasePigeon m_basePigeon;

    public PigeonSubsystem(BasePigeon basePigeon) {
        m_basePigeon = basePigeon;
    }
  
    public double getYaw() { return m_basePigeon.getYaw(); }
    public double getPitch() { return m_basePigeon.getPitch(); }
    public double getRoll() { return m_basePigeon.getRoll(); }
    
    public void setYaw(double yaw) { m_basePigeon.setYaw(yaw, 10); }
    public void addYaw(double yaw) { m_basePigeon.addYaw(yaw, 10); }
    public void setYawToCompass() { m_basePigeon.setYawToCompass(10); }
    public void setAccumZ(double accumZ) { m_basePigeon.setAccumZAngle(accumZ, 10); }
    public abstract boolean getFault();
    
    public double getCompass() { return m_basePigeon.getCompassHeading(); }
    public double getAccumZ() { 
        double[] accums = new double[3];
        m_basePigeon.getAccumGyro(accums);
        return accums[2];
    }
    public double[] getRawGyros() { 
        double[] gyrs = new double[3];
        m_basePigeon.getRawGyro(gyrs);
        return gyrs;
    }
    public int getUpTime() { return m_basePigeon.getUpTime(); }
    public double getTemp() { return m_basePigeon.getTemp(); }

    public abstract String getFaultMessage();
}
