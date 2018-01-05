
package org.usfirst.frc.team217.robot;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

	/* talons for arcade drive */
	WPI_TalonSRX _frontLeftMotor = new WPI_TalonSRX(11); 		/* device IDs here (1 of 2) */
	WPI_TalonSRX _frontRightMotor = new WPI_TalonSRX(14);

	/* extra talons for six motor drives */
	WPI_VictorSPX _leftSlave1 = new WPI_VictorSPX(13);
	WPI_VictorSPX _rightSlave1 = new WPI_VictorSPX(15);
	WPI_VictorSPX _leftSlave2 = new WPI_VictorSPX(16);
	WPI_VictorSPX _rightSlave2 = new WPI_VictorSPX(17);
	
	
	DifferentialDrive _drive = new DifferentialDrive(_frontLeftMotor, _frontRightMotor);
	
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
    	double forward = _joy.getY(); // logitech gampad left X, positive is forward
    	double turn = _joy.getZ(); //logitech gampad right X, positive means turn right
    	_drive.arcadeDrive(forward, turn);
    }
}
