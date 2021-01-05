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
    enum Devices {
        TalonSRX,
        VictorSPX,
        CANifier,
        PigeonIMU,
        CANCoder,
        TalonFX 
        ;

        /* Get next item in enum */
        public Devices next() {
            if(this == TalonFX) return TalonSRX;
            return values()[ordinal() + 1];
        }
        /* Get previous item in enum */
        public Devices previous() {
            if(this == TalonSRX) return TalonFX;
            return values()[ordinal() - 1];
        }
    }
    Devices _currentDevice = Devices.TalonSRX;

    /** Hardware */
    TalonSRX  _talon = new WPI_TalonSRX(1);
    VictorSPX  _victor = new WPI_VictorSPX(2);
    PigeonIMU  _pigeon = new PigeonIMU(3);
    CANifier  _canifier = new CANifier(4);
    CANCoder _canCoder = new CANCoder(5);
    TalonFX _fx = new TalonFX(6);
    
    Joystick _joy = new Joystick(0);
    
    /** Configs object with all four Configurations defined in Configs.java */
    Configs _custom_configs = new Configs();
    
    /** Track current and previous button state to detect single press button event */
    boolean[] _previousBtns = { false, false, false, false, false, 
                                false, false, false, false, false};
    int _previousPov = 0;

    /**
     * Run Forever in TeleOperated Mode
     */
    public void teleopPeriodic()
    {
        /* Get gamepad buttons and update current buttons container */
        boolean[] _btns = new boolean[_previousBtns.length];
        for (int i = 1; i < _previousBtns.length; ++i)
            _btns[i] = _joy.getRawButton(i);
        
        int pov = _joy.getPOV();
        boolean leftArrow = (pov == 270) && (_previousPov != 270);
        boolean rightArrow = (pov == 90) && (_previousPov != 90);

        if(leftArrow) {
            _currentDevice = _currentDevice.previous();
        }
        if(rightArrow) {
            _currentDevice = _currentDevice.next();
        }
        if(leftArrow || rightArrow) {
            switch (_currentDevice) {
                case TalonSRX: System.out.println("Selected TalonSRX"); break;
                case VictorSPX: System.out.println("Selected VictorSPX"); break;
                case CANifier: System.out.println("Selected CANifier"); break;
                case PigeonIMU: System.out.println("Selected PigeonIMU"); break;
                case CANCoder: System.out.println("Selected CANCoder"); break;
                case TalonFX: System.out.println("Selected TalonFX"); break;
            }
        }

        /* Read Configs (X-Button) */
        if (_btns[1] && !_previousBtns[1])
        {
            switch(_currentDevice) {
                case TalonSRX: {
                    System.out.printf("read talonsrx\n");
            
                    TalonSRXConfiguration read_talon = new TalonSRXConfiguration();
                    _talon.getAllConfigs(read_talon);
            
                    System.out.printf(read_talon.toString("_talon"));
                    break;
                }
                case VictorSPX: {
                    System.out.printf("read victorspx\n");
            
                    VictorSPXConfiguration read_victor = new VictorSPXConfiguration();
                    _victor.getAllConfigs(read_victor);
                        
                    System.out.printf(read_victor.toString("_victor"));
                    break;
                }
                case CANifier: {
                    System.out.printf("read canifier\n");
            
                    CANifierConfiguration read_canifier = new CANifierConfiguration();
                    _canifier.getAllConfigs(read_canifier);
            
                    System.out.printf(read_canifier.toString("_canifier"));
                    break;
                }
                case PigeonIMU: {
                    System.out.printf("read pigeonimu\n");
            
                    PigeonIMUConfiguration read_pigeon = new PigeonIMUConfiguration();
                    _pigeon.getAllConfigs(read_pigeon);
            
                    System.out.printf(read_pigeon.toString("_pigeon"));
                    break;
                }
                case CANCoder: {
                    System.out.printf("read canCoder\n");
            
                    CANCoderConfiguration read_cancoder = new CANCoderConfiguration();
                    _canCoder.getAllConfigs(read_cancoder);
            
                    System.out.printf(read_cancoder.toString("_canCoder"));
                    break;
                }
                case TalonFX: {
                    System.out.printf("read talonfx\n");
            
                    TalonFXConfiguration read_fx = new TalonFXConfiguration();
                    _fx.getAllConfigs(read_fx);
            
                    System.out.printf(read_fx.toString("_fx"));
                    break;
                }
            }
        }
        /* Config all Devices with Custom Configs (Left-Bumper) */
        else if (_btns[5] && !_previousBtns[5])
        {
            System.out.printf("custom config start\n");
    
            _talon.configAllSettings(_custom_configs._talon);
            _victor.configAllSettings(_custom_configs._victor);
            _pigeon.configAllSettings(_custom_configs._pigeon);
            _canifier.configAllSettings(_custom_configs._canifier);
            _canCoder.configAllSettings(_custom_configs._canCoder);
            _fx.configAllSettings(_custom_configs._fx);
    
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
            _canCoder.configFactoryDefault();
            _fx.configFactoryDefault();
    
            System.out.printf("factory default finish\n");
        }
        /* Update previous button container with current button container, compare next loop */
        for (int i = 1; i < _previousBtns.length; ++i)
            _previousBtns[i] = _btns[i];
        _previousPov = pov;
    }
}
