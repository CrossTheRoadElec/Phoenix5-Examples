package org.usfirst.frc.team217.robot;

import com.ctre.phoenix.*;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.sensors.*;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;

public class Robot extends IterativeRobot {
    /** make a talon with deviceId 0 */
    TalonSRX  _talon = new TalonSRX(1);
    VictorSPX  _victor = new VictorSPX(2);
    PigeonIMU  _pigeon = new PigeonIMU(3);
    CANifier  _canifier = new CANifier(4);
    
    Joystick _joy = new Joystick(0);
    
    Configs _custom_configs = new Configs();
    
    boolean[] _btns = {false, false, false, false, false, false, false, false, false, false};
    
    /** hold the last button values from gamepad, this makes detecting on-press events trivial */
    boolean[] _btnsLast = {false, false, false, false, false, false, false, false, false, false};
 
    public void teleopPeriodic()
    {
        /* get all the buttons */
        for (int i = 1; i < _btnsLast.length; ++i)
            _btns[i] = _joy.getRawButton(i);
 
        if (_btns[1] && !_btnsLast[2])
        {
            System.out.printf("read talon\n");
    
            TalonSRXConfiguration read_talon = new TalonSRXConfiguration();
            _talon.getAllConfigs(read_talon);
    
            System.out.printf(read_talon.toString("_talon"));
        }
        else if (_btns[2] && !_btnsLast[2])
        {
            System.out.printf("read victor\n");
    
            VictorSPXConfiguration read_victor = new VictorSPXConfiguration();
            _victor.getAllConfigs(read_victor);
                
            System.out.printf(read_victor.toString("_victor"));
        }
        else if (_btns[3] && !_btnsLast[3])
        {
    
            System.out.printf("read pigeon\n");
    
            PigeonIMUConfiguration read_pigeon = new PigeonIMUConfiguration();
            _pigeon.getAllConfigs(read_pigeon);
    
            System.out.printf(read_pigeon.toString("_pigeon"));
    
        }
        else if (_btns[4] && !_btnsLast[4])
        {
            System.out.printf("read canifier\n");
    
            CANifierConfiguration read_canifier = new CANifierConfiguration();
            _canifier.getAllConfigs(read_canifier);
    
            System.out.printf(read_canifier.toString("_canifier"));
        }
        else if (_btns[5] && !_btnsLast[5])
        {
            System.out.printf("custom config start\n");
    
            _talon.configAllSettings(_custom_configs._talon);
            _victor.configAllSettings(_custom_configs._victor);
            _pigeon.configAllSettings(_custom_configs._pigeon);
            _canifier.configAllSettings(_custom_configs._canifier);
    
            System.out.printf("custom config finish\n");
        }
        else if (_btns[6] && !_btnsLast[6])
        {
            System.out.printf("factory default start\n");
    
            _talon.configFactoryDefault();
            _victor.configFactoryDefault();
            _pigeon.configFactoryDefault();
            _canifier.configFactoryDefault();
    
            System.out.printf("factory default finish\n");
        }
        for (int i = 1; i < _btnsLast.length; ++i)
            _btnsLast[i] = _btns[i];
    }
}
