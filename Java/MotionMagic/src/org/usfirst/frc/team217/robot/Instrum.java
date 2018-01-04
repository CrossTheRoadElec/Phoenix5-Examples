package org.usfirst.frc.team217.robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Instrum {

	private static int _loops = 0;
	
	public static void Process(TalonSRX tal, StringBuilder sb)
	{
		/* smart dash plots */
    	SmartDashboard.putNumber("RPM", tal.getSelectedSensorVelocity(0));
    	SmartDashboard.putNumber("Pos",  tal.getSelectedSensorPosition(0));
    	SmartDashboard.putNumber("AppliedThrottle", (tal.getMotorOutputVoltage()/tal.getBusVoltage())*1023);
    	SmartDashboard.putNumber("ClosedLoopError", tal.getClosedLoopError(0));
    	if (tal.getControlMode() == ControlMode.MotionMagic) {
			//These API calls will be added in our next release.
    		SmartDashboard.putNumber("ActTrajVelocity", tal.getActiveTrajectoryVelocity());
    		SmartDashboard.putNumber("ActTrajPosition", tal.getActiveTrajectoryPosition());
    	}
    	/* periodically print to console */
        if(++_loops >= 10) {
        	_loops = 0;
        	System.out.println(sb.toString());
        }
        /* clear line cache */
        sb.setLength(0);
	}
}
