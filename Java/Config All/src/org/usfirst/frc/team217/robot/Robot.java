package org.usfirst.frc.team217.robot;

import com.ctre.phoenix.*;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.sensors.*;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;

/* This project may be used to test the getAllConfigs, configAllSettings, and configFactoryDefault
 * Functions. Note that this project requires at least firmware 3.11 on Victors/Talons
 * for full function. Also, if firmware greater than 0.41 on the pigeon and 0.42 on the canfier
 * isn't used, the pigeon/canifier won't retain configs on reboot.
 * Some recommended tests:
 *   1. Set to custom configs and then read configs. Confirm that read and write are the same.
 *   2. Set to factory default configs and then read configs and confirm they are what is expected.
 *   (RemoteFeedbackDevice and RemoteLimitSwitchSource on victorSPX read incorrectly in java after
 *   factory default)
 *   3. Set to custom configs and then restart devices. Confirm that all configs persist between
 *   reboots. (See above note about pigeon and CANifier firmware)
 */

public class Robot extends IterativeRobot {
    /** make a talon with deviceId 0 */
    TalonSRX  _talon = new TalonSRX(23);
    VictorSPX  _victor = new VictorSPX(2);
    PigeonIMU  _pigeon = new PigeonIMU(3);
    CANifier  _canifier = new CANifier(4);
    
    Joystick _joy = new Joystick(0);
    
    Configs _custom_configs = new Configs();
    
    boolean[] _btns = {false, false, false, false, false, false, false, false, false, false};
    boolean[] _btnsLast = {false, false, false, false, false, false, false, false, false, false};
    
    public void teleopPeriodic()
    {
        /* get gamepad buttons */
        for (int i = 1; i < _btnsLast.length; ++i)
            _btns[i] = _joy.getRawButton(i);
        /* on button1 press read talon configs */ 
        if (_btns[1] && !_btnsLast[2])
        {
            System.out.printf("read talon\n");
    
            TalonSRXConfiguration read_talon = new TalonSRXConfiguration();
            
            _talon.getAllConfigs(read_talon);
    
            System.out.printf(read_talon.toString("_talon"));
        }
        /* on button2 press read victor configs */
        else if (_btns[2] && !_btnsLast[2])
        {
            System.out.printf("read victor\n");
    
            VictorSPXConfiguration read_victor = new VictorSPXConfiguration();
            _victor.getAllConfigs(read_victor);
                
            System.out.printf(read_victor.toString("_victor"));
        }
        /* on button3 press read pigeon configs */
        else if (_btns[3] && !_btnsLast[3])
        {
    
            System.out.printf("read pigeon\n");
    
            PigeonIMUConfiguration read_pigeon = new PigeonIMUConfiguration();
            _pigeon.getAllConfigs(read_pigeon);
    
            System.out.printf(read_pigeon.toString("_pigeon"));
    
        }
        /* on button4 press read canifier configs */
        else if (_btns[4] && !_btnsLast[4])
        {
            System.out.printf("read canifier\n");
    
            CANifierConfiguration read_canifier = new CANifierConfiguration();
            _canifier.getAllConfigs(read_canifier);
    
            System.out.printf(read_canifier.toString("_canifier"));
        }
        /* on button5 press set custom configs */
        else if (_btns[5] && !_btnsLast[5])
        {
            System.out.printf("custom config start\n");
    
            _talon.configAllSettings(_custom_configs._talon);
            _victor.configAllSettings(_custom_configs._victor);
            _pigeon.configAllSettings(_custom_configs._pigeon);
            _canifier.configAllSettings(_custom_configs._canifier);
    
            System.out.printf("custom config finish\n");
        }
        /* on button6 press set factory default */
        else if (_btns[6] && !_btnsLast[6])
        {
            System.out.printf("factory default start\n");
    
            _talon.configFactoryDefault();
            _victor.configFactoryDefault();
            _pigeon.configFactoryDefault();
            _canifier.configFactoryDefault();
    
            System.out.printf("factory default finish\n");
        }
        /* set last presses */
        for (int i = 1; i < _btnsLast.length; ++i)
            _btnsLast[i] = _btns[i];
    }
}
