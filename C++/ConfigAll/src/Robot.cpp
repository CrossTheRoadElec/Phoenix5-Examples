
#include "WPILib.h"
#include "Configs.h"
#include "PrintFunctions.h"

class Robot: public IterativeRobot {
private:

	TalonSRX * _talon = new TalonSRX(1);
	VictorSPX * _victor = new VictorSPX(9);
	//PigeonIMU * _pigeon = new PigeonIMU(2);
	//CANifier * _canifier = new CANifier(3);
	Joystick * _joy = new Joystick(0);
	std::string _sb;

	configs _custom_configs;

	bool _button1_last = false;
	bool _button2_last = false;
	bool _button3_last = false;
	bool _button4_last = false;
	bool _button5_last = false;
	bool _button6_last = false;

	void RobotInit() {

	}

	/**
	 * This function is called periodically during operator control
	 */
	void TeleopPeriodic() {
		//std::cout << "Running" << std::endl;
		bool button1 = _joy->GetRawButton(1); //Button A on xbox style controllers, read talon part 1
		bool button2 = _joy->GetRawButton(2); //Button B on xbox style controllers, read talon part 2
		bool button3 = _joy->GetRawButton(3); //Button X on xbox style controllers, read victor part 1
		bool button4 = _joy->GetRawButton(4); //Button Y on xbox style controllers, read victor part 2
		bool button5 = _joy->GetRawButton(5); //Button Left Bumper on xbox style controllers, custom configs
		bool button6 = _joy->GetRawButton(6); //Button Right Bumper on xbox style controllers, API default

        
        if(button1 && !_button1_last) {
			std::cout << "read talon part 1" << std::endl;
			
			TalonSRXConfiguration read_talon;
			_talon->GetAllConfigs(read_talon);

			PrintTalonConfigPart1(read_talon);

		}
		else if(button2 && !_button2_last) {
			std::cout << "read talon part 2" << std::endl;
			
			TalonSRXConfiguration read_talon;
			_talon->GetAllConfigs(read_talon);

			PrintTalonConfigPart2(read_talon);

		}
		else if(button3 && !_button3_last) {
			std::cout << "read victor part 1" << std::endl;
			
			VictorSPXConfiguration read_victor;
			_victor->GetAllConfigs(read_victor);
			
			PrintVictorConfigPart1(read_victor);

		}
		else if(button4 && !_button4_last) {
			std::cout << "read victor part 2" << std::endl;
			
			VictorSPXConfiguration read_victor;
			_victor->GetAllConfigs(read_victor);
			
			PrintVictorConfigPart2(read_victor);

		}
		else if(button5 && !_button5_last) {
			std::cout << "custom config start" << std::endl;

			_talon->ConfigAllSettings(_custom_configs._talon);
			_victor->ConfigAllSettings(_custom_configs._victor);
			std::cout << "custom config finish" << std::endl;
			//_pigeon->ConfigAllSettings(custom_configs._pigeon);	
			//_canifier->ConfigAllSettings(custom_configs._canifier);	

		}
		else if(button6 && !_button6_last) {
			std::cout << "factory default start" << std::endl;
			_talon->ConfigFactoryDefault();
        	_victor->ConfigFactoryDefault();
        	std::cout << "factory default finish" << std::endl;
			//_pigeon->ConfigFactoryDefault();
        	//_canifier->ConfigFactoryDefault();

		}
		_button1_last = button1;
		_button2_last = button2;
		_button3_last = button3;
		_button4_last = button4;
		_button5_last = button5;
		_button6_last = button6;
	}

};

START_ROBOT_CLASS(Robot)
