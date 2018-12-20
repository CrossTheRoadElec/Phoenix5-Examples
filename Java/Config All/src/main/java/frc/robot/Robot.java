/**
 * Phoenix Software License Agreement
 *
 * Copyright (C) Cross The Road Electronics.  All rights
 * reserved.
 * 
 * Cross The Road Electronics (CTRE) licenses to you the right to 
 * use, publish, and distribute copies of CRF (Cross The Road) firmware files (*.crf) and 
 * Phoenix Software API Libraries ONLY when in use with CTR Electronics hardware products.
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
 * The Config All example project may be used to test the getAllConfigs, configAllSettings, 
 * and configFactoryDefault functions.
 * Some recommended tests:
 *  1. Set to custom configs and then read configs. Confirm that read and write are the same.
 *  2. Set to factory default configs and then read configs and confirm they are what is expected.
 *  (RemoteFeedbackDevice and RemoteLimitSwitchSource on victorSPX read incorrectly in java after
 *  factory default)
 *  3. Set to custom configs and then restart devices. Confirm that all configs persist between
 *  reboots. (Supported Firmware Version Details found below)
 * 
 * Controls:
 * Button 1: Read and Print Talon SRX Configs
 * Button 2: Read and Print Victor SPX Configs
 * Button 3: Read and Print Pigeon IMU Configs
 * Button 4: Read and Print CANifier Configs
 * Button 5: Config all 4 devices with Custom configs (Configs.java)
 * Button 6: Factory Default all Configs on all 4 devices
 * 
 * Supported Version:
 *	- Talon SRX: 4.0
 * 	- Victor SPX: 4.0
 * 	- Pigeon IMU: 4.0
 * 	- CANifier: 4.0
 */
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;

import com.ctre.phoenix.*;
import com.ctre.phoenix.sensors.*;
import com.ctre.phoenix.motorcontrol.can.*;

public class Robot extends TimedRobot {
    /** Hardware */
    TalonSRX  _talon = new TalonSRX(1);
    VictorSPX  _victor = new VictorSPX(2);
    PigeonIMU  _pigeon = new PigeonIMU(3);
    CANifier  _canifier = new CANifier(0);
    
    Joystick _joy = new Joystick(0);
    
    /** Configs object with all four Configurations defined in Configs.java */
    Configs _custom_configs = new Configs();
    
    /** Track current and previous button state to detect single press button event */
    boolean[] _previousBtns = { false, false, false, false, false, 
                                false, false, false, false, false};
    /**
     * Run Forever in TeleOperated Mode
     */
    public void teleopPeriodic()
    {
        /* Get gamepad buttons and update current buttons container */
        boolean[] _btns = new boolean[_previousBtns.length];
        for (int i = 1; i < _previousBtns.length; ++i)
            _btns[i] = _joy.getRawButton(i);
            
        /* Read Talon SRX Configs (X-Button) */
        if (_btns[1] && !_previousBtns[1])
        {
            System.out.printf("read talon\n");
    
            TalonSRXConfiguration read_talon = new TalonSRXConfiguration();
            _talon.getAllConfigs(read_talon);
    
            System.out.printf(read_talon.toString("_talon"));
        }
        /* Read Victor SPX Configs (A-Button) */
        else if (_btns[2] && !_previousBtns[2])
        {
            System.out.printf("read victor\n");
    
            VictorSPXConfiguration read_victor = new VictorSPXConfiguration();
            _victor.getAllConfigs(read_victor);
                
            System.out.printf(read_victor.toString("_victor"));
        }
        /* Read Pigeon IMU configs (B-Button) */
        else if (_btns[3] && !_previousBtns[3])
        {
            System.out.printf("read pigeon\n");
    
            PigeonIMUConfiguration read_pigeon = new PigeonIMUConfiguration();
            _pigeon.getAllConfigs(read_pigeon);
    
            System.out.printf(read_pigeon.toString("_pigeon"));
        }
        /* Read CANifier Configs (Y-Button) */
        else if (_btns[4] && !_previousBtns[4])
        {
            System.out.printf("read canifier\n");
    
            CANifierConfiguration read_canifier = new CANifierConfiguration();
            _canifier.getAllConfigs(read_canifier);
    
            System.out.printf(read_canifier.toString("_canifier"));
        }
        /* Config all Devices with Custom Configs (Left-Bumper) */
        else if (_btns[5] && !_previousBtns[5])
        {
            System.out.printf("custom config start\n");
    
            _talon.configAllSettings(_custom_configs._talon);
            _victor.configAllSettings(_custom_configs._victor);
            _pigeon.configAllSettings(_custom_configs._pigeon);
            _canifier.configAllSettings(_custom_configs._canifier);
    
            System.out.printf("custom config finish\n");
        }
        /* Factory Default all Devices (Right-Bumper) */
        else if (_btns[6] && !_previousBtns[6])
        {
            System.out.printf("factory default start\n");
    
            _talon.configFactoryDefault();
            _victor.configFactoryDefault();
            _pigeon.configFactoryDefault();
            _canifier.configFactoryDefault();
    
            System.out.printf("factory default finish\n");
        }
        /* Update previous button container with current button container, compare next loop */
        for (int i = 1; i < _previousBtns.length; ++i)
            _previousBtns[i] = _btns[i];
    }
}
