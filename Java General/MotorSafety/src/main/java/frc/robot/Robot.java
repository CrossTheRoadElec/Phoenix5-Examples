/**
 * Phoenix Software License Agreement
 *
 * Copyright (C) Cross The Road Electronics.  All rights
 * reserved.
 * 
 * Cross The Road Electronics (CTRE) licenses to you the right to 
 * use, publish, and distribute copies of CRF (Cross The Road) firmware files (*.crf) and 
 * Phoenix Software API Libraries ONLY when in use with CTR Electronics hardware products
 * as well as the FRC roboRIO when in use in FRC Competition.
 * 
 * THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT
 * WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT
 * LIMITATION, ANY WARRANTY OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * CROSS THE ROAD ELECTRONICS BE LIABLE FOR ANY INCIDENTAL, SPECIAL, 
 * INDIRECT OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF
 * PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
 * BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE
 * THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER
 * SIMILAR COSTS, WHETHER ASSERTED ON THE BASIS OF CONTRACT, TORT
 * (INCLUDING NEGLIGENCE), BREACH OF WARRANTY, OR OTHERWISE
 */

/**
 * Description:
 * Helpful examle explaining WPI's motor safety feature.
 * The goal is to auto-neutral your motors if your application stops calling set().
 *
 * Motor safety features can be a trap for teams where:
 *  - They unknowingly rely on this because they don't explicitly turn off motors at end of auton.
 *      Then when they disable it to workaround a problem, auton robot drives into a wall.
 *  - This can potentioally cause erroneuos motor disables due to intermittent
 *       lags in loop time (historically caused by excessive printing, logging, or competition WiFi lag).
 * 
 * Some tips:
 * - Always have a team member ready to disable the robot via Driver Station!
 * - Neutral your motor outputs explicitly in your disabled loop.
 * - Ensure you set motor output to zero when actions are complete.
 * - Use motor safety in contexts that makes sense, i.e. during source-level breakpoint/debugging.
 * 
 * Controls:
 * Left Joystick Y-Axis: Drive robot in forward and reverse direction
 * Right Joystick X-Axis: Turn robot in right and left direction
 * Button 1: Stops updating the drive train.  If safeties are on, this should neutral the motor and trigger
 *      a Driver Station message. If safeties are off MOTORS WILL CONTINUE LAST COMMAND.
 * 
 */
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.can.*;

import frc.robot.sim.PhysicsSim;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  /* Master Talons for arcade drive */
  WPI_TalonSRX _left = new WPI_TalonSRX(2);
  WPI_TalonSRX _rght = new WPI_TalonSRX(1);

  /* Construct drivetrain by providing master motor controllers */
  DifferentialDrive _drive = new DifferentialDrive(_left, _rght);

  Joystick _joy = new Joystick(0);  /* Joystick for control */

  int _loops = 0; // slow print to the DS

  @Override
  public void simulationInit() {
    PhysicsSim.getInstance().addTalonSRX(_left, 0.75, 4000);
    PhysicsSim.getInstance().addTalonSRX(_rght, 0.75, 4000, true);
  }
  @Override
  public void simulationPeriodic() {
    PhysicsSim.getInstance().run();
  }

  @Override
  public void teleopInit() {

    /* Motor controllers default motor safety OFF.
        WPI drive trains default motor safety ON.
        Experiment with different enables below.... */
    //_left.setSafetyEnabled(true);
    //_rght.setSafetyEnabled(true);
    //_drive.setSafetyEnabled(false);


    /* Factory Default all hardware to prevent unexpected behaviour */
    _left.configFactoryDefault();
    _rght.configFactoryDefault();

    /**
     * Drive robot forward and make sure all motors spin the correct way. Toggle
     * booleans accordingly....
     */
    _left.setInverted(false); // <<<<<< Adjust this until robot drives forward when stick is forward
    _rght.setInverted(true); // <<<<<< Adjust this until robot drives forward when stick is forward

    /*
     * diff drive assumes (by default) that right side must be negative to move
     * forward. Change to 'false' so positive/green-LEDs moves robot forward
     */
    _drive.setRightSideInverted(false); // do not change this
  }

  @Override
  public void teleopPeriodic() {
    /* Gamepad processing */
    double forward = -1.0 * _joy.getY(); // Sign this so forward is positive
    double turn = +1.0 * _joy.getZ(); // Sign this so right is positive

    /**
     * Print the joystick values to sign them, comment out this line after checking
     * the joystick directions.
     */
    if (++_loops >= 10) { /* slow print to the DS */
      _loops = 0;
      System.out.println("JoyY:" + forward + "  turn:" + turn);
    }


    if (_joy.getRawButton(1)) {
        /* if button 0 is pressed, stop calling arcadeDrive, which calls set() */
    } else {
      /* button is not held, update the drivetrain like normal */
      _drive.arcadeDrive(forward, turn);
    }
  }
}
