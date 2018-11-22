/* This project may be used to test the GetAllConfigs, ConfigAllSettings, and ConfigFactoryDefault
 * Functions. Note that this project requires at least firmware 3.11 on Victors/Talons
 * for full function. Also, if firmware greater than 0.41 on the pigeon and 0.42 on the canfier
 * isn't used, the pigeon/canifier won't retain configs on reboot.
 * Some recommended tests:
 *   1. Set to custom configs and then read configs. Confirm that read and write are the same.
 *   2. Set to factory default configs and then read configs and confirm they are what is expected.
 *   3. Set to custom configs and then restart devices. Confirm that all configs persist between
 *   reboots. (See above note about pigeon and CANifier firmware)
 */
#include <string>

#include "frc/WPILib.h"
#include "ctre/Phoenix.h"
#include "Configs.h"

using namespace frc;

class Robot: public TimedRobot {
private:
	/* Hardware */
	TalonSRX * _talon = new TalonSRX(23);
	VictorSPX * _victor = new VictorSPX(2);
	PigeonIMU * _pigeon = new PigeonIMU(3);
	CANifier * _canifier = new CANifier(4);
	Joystick * _joy = new Joystick(0);

	/* Config Class */
	configs _custom_configs;

	/* Track Button state */
	bool _button1_last = false;
	bool _button2_last = false;
	bool _button3_last = false;
	bool _button4_last = false;
	bool _button5_last = false;
	bool _button6_last = false;

	void RobotInit() {
		/* Do nothing for init */
	}

	/**
	 * This function is called periodically during operator control
	 */
	void TeleopPeriodic() {
		/* get gamepad buttons */
        bool button1 = _joy->GetRawButton(1); // read talon
		bool button2 = _joy->GetRawButton(2); // read victor
		bool button3 = _joy->GetRawButton(3); // read pigeon
		bool button4 = _joy->GetRawButton(4); // read canifier
		bool button5 = _joy->GetRawButton(5); // custom configs
		bool button6 = _joy->GetRawButton(6); // factory default

        /* on button1 press read talon configs */ 
        if(button1 && !_button1_last) {
			printf("read talon\n");
			
			TalonSRXConfiguration read_talon;
			_talon->GetAllConfigs(read_talon);

			printf(read_talon.toString("_talon").c_str());
		}
        /* on button2 press read victor configs */ 
		else if(button2 && !_button2_last) {
			printf("read victor\n");

			VictorSPXConfiguration read_victor;
			_victor->GetAllConfigs(read_victor);

			printf(read_victor.toString("_victor").c_str());
		}
        /* on button3 press read pigeon configs */ 
		else if(button3 && !_button3_last) {
			printf("read pigeon\n");

			PigeonIMUConfiguration read_pigeon;
			_pigeon->GetAllConfigs(read_pigeon);

			printf(read_pigeon.toString("_pigeon").c_str());
		}
        /* on button4 press read canifier configs */ 
		else if(button4 && !_button4_last) {
			printf("read canifier\n");

			CANifierConfiguration read_canifier;
			_canifier->GetAllConfigs(read_canifier);

			printf(read_canifier.toString("_canifier").c_str());
		}
        /* on button5 press set custom configs */ 
		else if(button5 && !_button5_last) {
			printf("custom config start\n");

			_talon->ConfigAllSettings(_custom_configs._talon);
			_victor->ConfigAllSettings(_custom_configs._victor);
			_pigeon->ConfigAllSettings(_custom_configs._pigeon);
			_canifier->ConfigAllSettings(_custom_configs._canifier);

			printf("custom config finish\n");
		}
        /* on button6 press set factory default */ 
		else if(button6 && !_button6_last) {
			printf("factory default start\n");

			_talon->ConfigFactoryDefault();
        	_victor->ConfigFactoryDefault();
			_pigeon->ConfigFactoryDefault();
        	_canifier->ConfigFactoryDefault();
        	
            printf("factory default finish\n");
		}

        /* set last presses */        
		_button1_last = button1;
		_button2_last = button2;
		_button3_last = button3;
		_button4_last = button4;
		_button5_last = button5;
		_button6_last = button6;
	}
};

START_ROBOT_CLASS(Robot)
