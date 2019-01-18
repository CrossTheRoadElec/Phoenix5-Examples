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
 * The PigeonRemoteSensor example demonstrates using a PigeonIMU as a remote sensor
 * Tested with Logitech F350 USB Gamepad inserted into Driver Station
 * 
 * Be sure to configure the correct motor controller and pigeon
 * 
 * Controls:
 * X, B Buttons: Cycle between yaw, pitch, and roll as the selected filter for the motor controller
 * A Button: Zero yaw
 * Y Button: Start printing information every 50 loops
 * Left, Right Bumpers: Cycle between printing YPR information, and any other information pigeon has
 * Left joystick up/down: Throttle for the Robot
 * Right joystick left/right: Turn for the Robot
 * 
 */

package frc.robot;

import edu.wpi.first.wpilibj.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.sensors.PigeonIMU;

/***    _____  _                       _____                _ 
 *     |  __ \| |                     |  __ \              | |
 *     | |__) | | ___  __ _ ___  ___  | |__) |___  __ _  __| |
 *     |  ___/| |/ _ \/ _` / __|/ _ \ |  _  // _ \/ _` |/ _` |
 *     | |    | |  __/ (_| \__ \  __/ | | \ \  __/ (_| | (_| |
 *     |_|    |_|\___|\__,_|___/\___| |_|  \_\___|\__,_|\__,_|              
 * 
 * The example has all the tools needed to determine if the pigeon is in a good position on the robot
 *    i.e. it will measure the desired values under all circumstances the robot will experience.
 * 
 * The first thing you should do is determine if the placement of the pigeon is close enough to the center of rotation
 *  - The Pigeon IMU is a sensor that is impacted by excessive and sustained g-forces. Keeping it near the center of rotation
 *    will prevent this excessive and sustained g-force while the robot is rotating.
 * The procedure for this is as follows:
 *  1. Drive the robot into an immovable flat obstacle, such as a wall
 *  2. Zero the yaw and accumZ by pressing the A button
 *  3. Drive the robot in a zero turn at max speed for about 30 seconds
 *  4. Drive the robot in the opposite direction for about 30 seconds or until yaw is around 0
 *  5. Drive back up to the immovable object, and measure the yaw value and accum Z value
 *  6. If yaw is incorrect and accumZ is correct, move the IMU closer to Centor of rotation and repeat
 * 
 * The second thing you should do is temperature-calibrate the pigeon
 *  - Follow the guide in the documentation Bring-Up Pigeon for this 
 *      https://phoenix-documentation.readthedocs.io/en/latest/ch11_BringUpPigeon.html#temperature-calibration
 * 
 * A quick guide on how to temperature calibrate is below:
 *  1. Ensure pigeon is cool before beginning temperature calibration. This can be confirmed with a self test
 *  2. Enter temperature calibration mode. This is done either using the API or using Phoenix Tuner
 *  3. Heat the pigeon.
 *  4. Once the pigeon has seen a sufficient range of temperatures, it will momentarily blink green, then cleanly boot-calibrate.
 */
public class Robot extends TimedRobot {
  
  PigeonIMU _pidgey;
  TalonSRX _pigeonTalon = new TalonSRX(2);
  BaseMotorController _motorController = new TalonSRX(1);
  boolean pigeonOverCAN = true;

  BaseMotorController leftSide = _motorController;
  BaseMotorController rightSide = new VictorSPX(2);

  Joystick _joy = new Joystick(0);

  int pigeonAxisUsed = 0;
  int pigeonInformationUsed = 0;
  boolean printInformation = false;

  int printCounter = 0;

  @Override
  public void robotInit() {
    if(pigeonOverCAN)
      _pidgey = new PigeonIMU(1);
    else
      _pidgey = new PigeonIMU(_pigeonTalon);

    _motorController.configFactoryDefault();
    /** Configure filter to use Yaw for the moment */
    _motorController.configRemoteFeedbackFilter(_pidgey.getDeviceID() , 
                                                pigeonOverCAN ? RemoteSensorSource.Pigeon_Yaw : RemoteSensorSource.GadgeteerPigeon_Yaw, 
                                                0);
    _motorController.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
  }

  @Override
  public void robotPeriodic() {
    if(_joy.getRawButtonPressed(2))
    {
      /** Zero yaw, this has to be done using the pigeon, not the motor controller */
      _pidgey.setYaw(0);
      _pidgey.setAccumZAngle(0);
      System.out.println("============================");
      System.out.println("Yaw and accumulated Z zero'ed");
      System.out.println("============================");
      System.out.println();
    }

    if(_joy.getRawButtonPressed(1) || _joy.getRawButtonPressed(3))
    {
      if(_joy.getRawButton(1)) pigeonAxisUsed--;
      else pigeonAxisUsed++;

      if(pigeonAxisUsed < 0)
        pigeonAxisUsed = 2;
      else if(pigeonAxisUsed > 2)
        pigeonAxisUsed = 0;

      System.out.println("============================");
      /** Increment feedback filter to use other degree from pigeon  */
      switch(pigeonAxisUsed)
      {
        case 0:
          _motorController.configRemoteFeedbackFilter(_pidgey.getDeviceID() , 
                                                      pigeonOverCAN ? RemoteSensorSource.Pigeon_Yaw : RemoteSensorSource.GadgeteerPigeon_Yaw, 
                                                      0);
          System.out.println("Motor controller filtering for pigeon Yaw");
          break;
        case 1:
          _motorController.configRemoteFeedbackFilter(_pidgey.getDeviceID() , 
                                                      pigeonOverCAN ? RemoteSensorSource.Pigeon_Pitch : RemoteSensorSource.GadgeteerPigeon_Pitch, 
                                                      0);
          System.out.println("Motor controller filtering for pigeon Pitch");
          break;
        case 2:
          _motorController.configRemoteFeedbackFilter(_pidgey.getDeviceID() , 
                                                      pigeonOverCAN ? RemoteSensorSource.Pigeon_Roll : RemoteSensorSource.GadgeteerPigeon_Roll, 
                                                      0);
          System.out.println("Motor controller filtering for pigeon Roll");
          break;
      }
      System.out.println("============================");
      System.out.println();
    }

    if(_joy.getRawButtonPressed(4))
    {
      /** Toggle printing information */
      printInformation = !printInformation;
      System.out.println("============================");
      System.out.println("Printing is " + (printInformation ? "Enabled" : "Disabled"));
      System.out.println("============================");
      System.out.println();
    }

    if(_joy.getRawButtonPressed(5) || _joy.getRawButtonPressed(6))
    {
      if(_joy.getRawButton(5)) pigeonInformationUsed--;
      else pigeonInformationUsed++;

      if(pigeonInformationUsed < 0)
        pigeonInformationUsed = 7;
      else if(pigeonInformationUsed > 7)
        pigeonInformationUsed = 0;

      /** Toggle using ypr or quaternions/accum signals */
      switch(pigeonInformationUsed)
      {
        case 0:
          /** Use YPR */
          System.out.println("============================");
          System.out.println("Reading YPR from Pigeon");
          System.out.println("============================");
          break;
        case 1:
          /** Use Quaternion */
          System.out.println("============================");
          System.out.println("Reading Quaternion from Pigeon");
          System.out.println("============================");
          break;
        case 2:
          /** Use Accum Gyro */
          System.out.println("============================");
          System.out.println("Reading Accum Gyro from Pigeon");
          System.out.println("============================");
          break;
        case 3:
          /** Use Biased Accel */
          System.out.println("============================");
          System.out.println("Reading Biased Accel from Pigeon");
          System.out.println("============================");
          break;
        case 4:
          /** Use Raw Gyro */
          System.out.println("============================");
          System.out.println("Reading Raw Gyro from Pigeon");
          System.out.println("============================");
          break;
        case 5:
          /** Use Accelerometer Angles */
          System.out.println("============================");
          System.out.println("Reading Accelerometer from Pigeon");
          System.out.println("============================");
          break;
        case 6:
          /** Use Biased Magnetometer */
          System.out.println("============================");
          System.out.println("Reading Biased Magnetometer from Pigeon");
          System.out.println("============================");
          break;
        case 7:
          /** Use Raw Magnetometer */
          System.out.println("============================");
          System.out.println("Reading Raw Magnetometer from Pigeon");
          System.out.println("============================");
          break;
      }
    }

    if(printInformation && printCounter++ > 50)
    {
      printCounter = 0;
      switch(pigeonInformationUsed)
      {
        case 0:
          /** Printing YPR Values with Talon */
          double[] ypr = new double[3];
          _pidgey.getYawPitchRoll(ypr);

          System.out.println("Printing Yaw/Pitch/Roll");
          System.out.println("Pigeon value: " + ypr[pigeonAxisUsed]);
          System.out.println("Selected Sensor Value: " + _motorController.getSelectedSensorPosition());
          System.out.println();
          break;
        case 1:
          /** Printing Quaternions */
          double[] quaternions = new double[4];
          _pidgey.get6dQuaternion(quaternions);
          System.out.println("Printing Quaternion Values");
          System.out.println("W: " + quaternions[0]);
          System.out.println("X: " + quaternions[1]);
          System.out.println("Y: " + quaternions[2]);
          System.out.println("Z: " + quaternions[3]);
          System.out.println();
          break;
        case 2:
          /** Printing Accumulated Gyro */
          double[] accumGyro = new double[3];
          _pidgey.getAccumGyro(accumGyro);
          System.out.println("X: " + accumGyro[0]);
          System.out.println("Y: " + accumGyro[1]);
          System.out.println("Z: " + accumGyro[2]);
          System.out.println();
          break;
        case 3:
          /** Printing Biased Accelerometer Angles */
          short[] biasedAccel = new short[3];
          _pidgey.getBiasedAccelerometer(biasedAccel);
          System.out.println("X: " + biasedAccel[0]);
          System.out.println("Y: " + biasedAccel[1]);
          System.out.println("Z: " + biasedAccel[2]);
          System.out.println();
          break;
        case 4:
          /** Printing Raw Gyro */
          double[] rawGyro = new double[3];
          _pidgey.getRawGyro(rawGyro);
          System.out.println("X: " + rawGyro[0]);
          System.out.println("Y: " + rawGyro[1]);
          System.out.println("Z: " + rawGyro[2]);
          System.out.println();
          break;
        case 5:
          /** Printing Accelerometer Angles */
          double[] accelAngles = new double[3];
          _pidgey.getAccelerometerAngles(accelAngles);
          System.out.println("X: " + accelAngles[0]);
          System.out.println("Y: " + accelAngles[1]);
          System.out.println("Z: " + accelAngles[2]);
          System.out.println();
          break;
        case 6:
          /** Printing Biased Magnetometer Angles */
          short[] biasedMagnet = new short[3];
          _pidgey.getBiasedMagnetometer(biasedMagnet);
          System.out.println("X: " + biasedMagnet[0]);
          System.out.println("Y: " + biasedMagnet[1]);
          System.out.println("Z: " + biasedMagnet[2]);
          System.out.println();
          break;
        case 7:
          /** Printing Raw Magnetometer Angles */
          short[] rawMagnet = new short[3];
          _pidgey.getRawMagnetometer(rawMagnet);
          System.out.println("X: " + rawMagnet[0]);
          System.out.println("Y: " + rawMagnet[1]);
          System.out.println("Z: " + rawMagnet[2]);
          System.out.println();
          break;
      }
    }
  }

  @Override
  public void autonomousInit() {
    
  }

  @Override
  public void autonomousPeriodic() {
    
  }

  @Override
  public void teleopPeriodic() {
    leftSide.set(ControlMode.PercentOutput, _joy.getRawAxis(1), DemandType.ArbitraryFeedForward, _joy.getRawAxis(2));
    rightSide.set(ControlMode.PercentOutput, _joy.getRawAxis(1), DemandType.ArbitraryFeedForward, -_joy.getRawAxis(2));
  }

  @Override
  public void testPeriodic() {
  }
}
