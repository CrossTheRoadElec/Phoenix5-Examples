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
 * PLEASE READ FIRST
 * 
 * Description:
 * The PigeonRemoteSensor example demonstrates:
 *  - how to get values from a PigeonIMU and
 *  - how to to use a PigeonIMU as a remote sensor
 * Tested with Logitech F350 USB Gamepad inserted into Driver Station
 * 
 * Be sure to configure the correct motor controller and pigeon
 * The example has all the tools needed to determine if the pigeon is in a good
 * position on the robot i.e. it will measure the desired values under all
 * circumstances the robot will experience.
 * 
 * The first thing you should do is determine if the placement of the pigeon is
 * close enough to the center of rotation - The Pigeon IMU is a sensor that is
 * impacted by excessive and sustained g-forces. Keeping it near the center of
 * rotation will prevent this excessive and sustained g-force while the robot is
 * rotating. The procedure for this is as follows: 1. Drive the robot into an
 * immovable flat obstacle, such as a wall 2. Zero the yaw and accumZ by
 * pressing the A button 3. Drive the robot in a zero turn at max speed for
 * about 30 seconds 4. Drive the robot in the opposite direction for about 30
 * seconds or until yaw is around 0 5. Drive back up to the immovable object,
 * and measure the yaw value and accum Z value 6. If yaw is incorrect and accumZ
 * is correct, move the IMU closer to Centor of rotation and repeat
 * 
 * The second thing you should do is temperature-calibrate the pigeon - Follow
 * the guide in the documentation Bring-Up Pigeon for this
 * https://phoenix-documentation.readthedocs.io/en/latest/ch11_BringUpPigeon.html#temperature-calibration
 * 
 * A quick guide on how to temperature calibrate is below: 1. Ensure pigeon is
 * cool before beginning temperature calibration. This can be confirmed with a
 * self test 2. Enter temperature calibration mode. This is done either using
 * the API or using Phoenix Tuner 3. Heat the pigeon. 4. Once the pigeon has
 * seen a sufficient range of temperatures, it will momentarily blink green,
 * then cleanly boot-calibrate.
 * 
 * Controls:
 * X, B Buttons: Cycle between yaw, pitch, and roll as the selected filter for the motor controller
 * A Button: Zero yaw
 * Y Button: Start printing information every 50 loops
 * Left, Right Bumpers: Cycle between printing YPR information, and any other information pigeon has
 * Left joystick up/down: Throttle for the Robot
 * Right joystick left/right: Turn for the Robot
 */

package frc.robot;

import edu.wpi.first.wpilibj.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.sensors.PigeonIMU;

public class Robot extends TimedRobot {

    /** pigeon instance, this is instantiated later, leave this alone */
    PigeonIMU _pidgey;

    /* pick one of these */
    TalonSRX _pigeonTalon = null;//new TalonSRX(0); /* Pigeon is ribbon cabled to this Talon */
    // TalonSRX _pigeonTalon = null; /* Pigeon is on CAN bus, so there is no Talon to create

    /**
     * if Pigeon is on CAN-bus, enter the device ID. its connected to Talon, this
     * does not matter
     */
    final int kPigeonID = 3;

    /**
     * If using remote talon features for drive train, provide the drivetrain
     * talons/victors. Otherwise set them to null. Create both or null both.
     * 
     * The RIGHT SIDE Talon is used to capture Pigeon values for Arc'ing / Differential-closed-looping.
     * 
     */
    // BaseMotorController leftSide = new TalonSRX(1); // using Talon for drive train
    // BaseMotorController rightSide = new VictorSPX(2); // using Talon for drive train
    BaseMotorController _leftSide = null; // no talons, just polling Pigeon for direct processing
    BaseMotorController _rghtSide = null; // no talons, just polling Pigeon for direct processing

    /**
     * Which remote filter to use in Talon to capture Pigeon (0 or 1). 
     * If you are not using Pigeon in your drietrain closed-loop, this doesn't matter.
     */
    final int kRemoteFilter = 0;

    /* joystick for control */
    Joystick _joy = new Joystick(0);

    int _axisSelection = 0; //!< [0,2] => [Yaw,Pitch,Roll]
    int _signalSelection = 0;  //!< [0,7] => What signal to print, see Instrum implem
    boolean _printEnable = true; //!< True => print signal values periodically

    /* timeouts for certain blocking actions */
    final int kTimeoutMs = 50;

    @Override
    public void robotInit() {
        /* create the pigeon */
        if (IsPigeonOnCanbus())
            _pidgey = new PigeonIMU(kPigeonID);
        else
            _pidgey = new PigeonIMU(_pigeonTalon);

        /*
         * if using the pigeon in your Talon closed loop, setup talon to capture Pigeon
         * data
         */
        if (_rghtSide != null) {
            /* factory default configs */
            _leftSide.configFactoryDefault();
            _rghtSide.configFactoryDefault();
            _pidgey.configFactoryDefault();


            /** Configure filter to use Yaw for the moment */
            if (kRemoteFilter == 0)
                _rghtSide.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0);
            else
                _rghtSide.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor1);

            /* config the Talon/Vic's remote sensor */
            SetupTalonRemoteSensorSource();

            /* pick the inverts, pushing stick forward should cause 
                green LEDs on all MCs, forward robot motion */
            _leftSide.setInverted(false); // <<<<<<<<<< adjust this 
            _rghtSide.setInverted(true); // <<<<<<<<<< adjust this 
        }
    }

    /**
     * helper routine to track if pigeon is on CAN or ribbon cable, this influences
     * how to setup devices
     */
    boolean IsPigeonOnCanbus() {
        if (_pigeonTalon == null)
            return true;
        return false;
    }
    /**
     * This configures the Talon to understand which Pigeon to gets values from.
     * If you are not using Pigeon as part of your Talon closed-loop, 
     * then this routine does not matter.
     */
    void SetupTalonRemoteSensorSource() {
        if (_rghtSide == null) {
            System.out.println("============================");
            System.out.println("Motor controller not selected, so remote filter is not setup.");
            System.out.println("============================");
            System.out.println();

        } else {

            RemoteSensorSource senSource = RemoteSensorSource.Pigeon_Yaw;
            String msg = "";

            switch (_axisSelection) {
                case 0:
                    if (IsPigeonOnCanbus())
                        senSource = RemoteSensorSource.Pigeon_Yaw;
                    else
                        senSource = RemoteSensorSource.GadgeteerPigeon_Yaw;
                    break;
                case 1:
                    if (IsPigeonOnCanbus())
                        senSource = RemoteSensorSource.Pigeon_Pitch;
                    else
                        senSource = RemoteSensorSource.GadgeteerPigeon_Pitch;
                    break;
                case 2:
                    if (IsPigeonOnCanbus())
                        senSource = RemoteSensorSource.Pigeon_Roll;
                    else
                        senSource = RemoteSensorSource.GadgeteerPigeon_Roll;
                    break;
            }
            _rghtSide.configRemoteFeedbackFilter(_pidgey.getDeviceID(), senSource, kRemoteFilter);

            System.out.println("============================");
            System.out.println("Motor controller filtering for " + senSource.toString());
            System.out.println("============================");
            System.out.println();
        }
    }

    @Override
    public void robotPeriodic() {

        /* button 2 will zero Pigeon */
        if (_joy.getRawButtonPressed(2)) {
            /** Zero yaw, this has to be done using the pigeon, not the motor controller */
            _pidgey.setYaw(0, kTimeoutMs);
            _pidgey.setAccumZAngle(0, kTimeoutMs);

            System.out.println("============================");
            System.out.println("Yaw and accumulated Z zero'ed");
            System.out.println("============================");
            System.out.println();
        }

        /* button 1 and 3 will change which Pigeon axis to display */
        if (_joy.getRawButtonPressed(1) || _joy.getRawButtonPressed(3)) {
            if (_joy.getRawButton(1))
                _axisSelection--;
            else
                _axisSelection++;
            /* some clever math to wrap 3 <=> 0 */
            _axisSelection = (_axisSelection + 3) % 3;

            /** Update Talon filter if using Talon closed-loop features */
            SetupTalonRemoteSensorSource();
        }

        /* button 4 will toggle the printing. */
        if (_joy.getRawButtonPressed(4)) {
            /** Toggle printing information */
            _printEnable = !_printEnable;
            System.out.println("============================");
            System.out.println("Printing is " + (_printEnable ? "Enabled" : "Disabled"));
            System.out.println("============================");
            System.out.println();
        }

        /* Shoulder buttons 5 and 6 will change the signal selection */
        if (_joy.getRawButtonPressed(5) || _joy.getRawButtonPressed(6)) {

            if (_joy.getRawButton(5))
                _signalSelection--;
            else
                _signalSelection++;

            /* some clever math to wrap 8 <=> 0 */
            _signalSelection = (_signalSelection + 8) % 8;

            /* print the selection change */
            Instrum.PrintSelection(_signalSelection);
        }

        /* print latest signal values (if printing enabled) */
        Instrum.Process(_pidgey, _rghtSide, _printEnable, _signalSelection, _axisSelection);
    }

    @Override
    public void teleopPeriodic() {
        /* get joystick values */
        double frwd = +1 * _joy.getRawAxis(1); /* signed so positive means drive forward (green LEDs) */
        double turn = -1 * _joy.getRawAxis(2); /* signed so positive means turn to the left */

        if (_rghtSide == null) {
            /* there are no talons in this setup */
        } else {
            /* drive the drivetrain assuming a simple diff drive. Alernatively a drivetrain class can be used. */
            _leftSide.set(ControlMode.PercentOutput, frwd, DemandType.ArbitraryFeedForward, -turn);
            _rghtSide.set(ControlMode.PercentOutput, frwd, DemandType.ArbitraryFeedForward, +turn);
        }
    }
}
