
package org.usfirst.frc.team217.robot;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

	/* talons for arcade drive */
	TalonSRX _frontLeftMotor = new TalonSRX(11); 		/* device IDs here (1 of 2) */
	TalonSRX _frontRightMotor = new TalonSRX(14);

	/* extra talons for six motor drives */
	VictorSPX _leftSlave1 = new VictorSPX(13);
	VictorSPX _rightSlave1 = new VictorSPX(15);
	VictorSPX _leftSlave2 = new VictorSPX(16);
	VictorSPX _rightSlave2 = new VictorSPX(17);
	
	
	DifferentialDrive _drive = new DifferentialDrive(_frontLeftMotor.getWPILIB_SpeedController(), 
			_frontRightMotor.getWPILIB_SpeedController());
	
	Joystick _joy = new Joystick(0);
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
    	/* take our extra talons and just have them follow the Talons updated in arcadeDrive */
    	_leftSlave1.follow(_frontLeftMotor);
    	_leftSlave2.follow(_frontLeftMotor);
    	_rightSlave1.follow(_frontRightMotor);
    	_rightSlave2.follow(_frontRightMotor);
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
    	double forward = _joy.getRawAxis(1); // logitech gampad left X, positive is forward
    	double turn = _joy.getRawAxis(2); //logitech gampad right X, positive means turn right
    	_drive.arcadeDrive(forward, turn);
    }
}
