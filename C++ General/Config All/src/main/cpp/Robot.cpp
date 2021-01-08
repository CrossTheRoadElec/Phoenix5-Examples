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
/* This project may be used to test the GetAllConfigs, ConfigAllSettings, and ConfigFactoryDefault
 * Functions. Note that this project requires at least firmware 3.11 on Victors/Talons
 * for full function. Also, if firmware greater than 0.41 on the pigeon and 0.42 on the canfier
 * isn't used, the pigeon/canifier won't retain configs on reboot.
 * Some recommended tests:
 *   1. Set to custom configs and then read configs. Confirm that read and write are the same.
 *   2. Set to factory default configs and then read configs and confirm they are what is expected.
 *   3. Set to custom configs and then restart devices. Confirm that all configs persist between
 *   reboots. (See above note about pigeon and CANifier firmware)
 *
 * Desktop simulation is supported if using WPILIB v2021.1.1-beta-4 or greater.
 * Otherwise comment out SimulationInit and SimulationPeriodic routines.
 * Enter "localhost" into Tuner's Server Address to interact with simulated devices.
 */
#include <string>

#include "frc/WPILib.h"
#include "ctre/Phoenix.h"
#include "Configs.h"

using namespace frc;

class Robot: public TimedRobot {
private:
	/* Hardware */
	TalonSRX * _talon = new WPI_TalonSRX(23);
	VictorSPX * _victor = new WPI_VictorSPX(2);
	PigeonIMU * _pigeon = new PigeonIMU(3);
	CANifier * _canifier = new CANifier(4);
	CANCoder * _canCoder = new CANCoder(5);
	TalonFX * _fx = new TalonFX(6);
	Joystick * _joy = new Joystick(0);

	/* Config Class */
	configs _custom_configs;

	/* Track Button state */
	bool _button1_last = false;
	bool _button5_last = false;
	bool _button6_last = false;
	bool _leftArrowLast = false;
	bool _rightArrowLast = false;

	const int Device_Count = 6;
	enum Device_Type{
		enumTalonSRX,
		enumVictorSPX,
		enumCANifier,
		enumPigeonIMU,
		enumCANCoder,
		enumTalonFX,
	} _selectedDevice;

	void RobotInit() {
		/* Do nothing for init */
	}

	/**
	 * This function is called periodically during operator control
	 */
	void TeleopPeriodic() {
		/* get gamepad buttons */
		int pov = _joy->GetPOV();
		bool leftArrow = pov == 270;
		bool rightArrow = pov == 90;
        bool button1 = _joy->GetRawButton(1); // read device
		bool button5 = _joy->GetRawButton(5); // custom configs
		bool button6 = _joy->GetRawButton(6); // factory default

		/* Change selected device based on directional input */
		if(leftArrow && ! _leftArrowLast) {
			int newDev = _selectedDevice;
			newDev--;
			if(newDev < 0) newDev = Device_Count - 1;
			_selectedDevice = (enum Device_Type)newDev;
		}
		if(rightArrow && ! _rightArrowLast) {
			int newDev = _selectedDevice;
			newDev++;
			if(newDev >= Device_Count) newDev = 0;
			_selectedDevice = (enum Device_Type)newDev;
		}
		/* Print selected device on change */
		if((leftArrow && ! _leftArrowLast) || (rightArrow && ! _rightArrowLast)) {
			switch(_selectedDevice) {
				case enumTalonSRX: printf("Selected TalonSRX\n"); break;
				case enumVictorSPX: printf("Selected VictorSPX\n"); break;
				case enumCANifier: printf("Selected CANifier\n"); break;
				case enumPigeonIMU: printf("Selected PigeonIMU\n"); break;
				case enumCANCoder: printf("Selected CANCoder\n"); break;
				case enumTalonFX: printf("Selected TalonFX\n"); break;
			}
		}
        /* on button1 press read talon configs */ 
        if(button1 && !_button1_last) {
			switch(_selectedDevice) {
				case enumTalonSRX: {
					printf("read talon\n");
					
					TalonSRXConfiguration read_talon;
					_talon->GetAllConfigs(read_talon);

					printf(read_talon.toString("_talon").c_str());
					break;
				}
				case enumVictorSPX: {
					printf("read victor\n");

					VictorSPXConfiguration read_victor;
					_victor->GetAllConfigs(read_victor);

					printf(read_victor.toString("_victor").c_str());
					break;
				}
				case enumCANifier: {
					printf("read canifier\n");

					CANifierConfiguration read_canifier;
					_canifier->GetAllConfigs(read_canifier);

					printf(read_canifier.toString("_canifier").c_str());
					break;
				}
				case enumPigeonIMU: {
					printf("read pigeon\n");

					PigeonIMUConfiguration read_pigeon;
					_pigeon->GetAllConfigs(read_pigeon);

					printf(read_pigeon.toString("_pigeon").c_str());
					break;
				}
				case enumCANCoder: {
					printf("read cancoder\n");

					CANCoderConfiguration read_cancoder;
					_canCoder->GetAllConfigs(read_cancoder);

					printf(read_cancoder.toString("_canCoder").c_str());
					break;
				}
				case enumTalonFX: {
					printf("read talonfx\n");

					TalonFXConfiguration read_talonfx;
					_fx->GetAllConfigs(read_talonfx);

					printf(read_talonfx.toString("_fx").c_str());
					break;
				}
				
				default: break;
			}
		}
        /* on button5 press set custom configs */ 
		else if(button5 && !_button5_last) {
			printf("custom config start\n");

			_talon->ConfigAllSettings(_custom_configs._talon);
			_victor->ConfigAllSettings(_custom_configs._victor);
			_pigeon->ConfigAllSettings(_custom_configs._pigeon);
			_canifier->ConfigAllSettings(_custom_configs._canifier);
			_canCoder->ConfigAllSettings(_custom_configs._canCoder);
			_fx->ConfigAllSettings(_custom_configs._fx);

			printf("custom config finish\n");
		}
        /* on button6 press set factory default */ 
		else if(button6 && !_button6_last) {
			printf("factory default start\n");

			_talon->ConfigFactoryDefault();
        	_victor->ConfigFactoryDefault();
			_pigeon->ConfigFactoryDefault();
        	_canifier->ConfigFactoryDefault();
			_canCoder->ConfigFactoryDefault();
			_fx->ConfigFactoryDefault();
        	
            printf("factory default finish\n");
		}

        /* set last presses */        
		_button1_last = button1;
		_button5_last = button5;
		_button6_last = button6;
		_leftArrowLast = leftArrow;
		_rightArrowLast = rightArrow;
	}
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif