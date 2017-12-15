#include "ServoHoldHeadingWithImu.h"

namespace CTRE {
namespace Motion {

ServoHoldHeadingWithImu::ServoHoldHeadingWithImu(CTRE::PigeonImu *pigeonImu, CTRE::Drive::IDrivetrain *driveTrain, CTRE::Drive::Styles::Basic selectedStyle,
		ServoParameters *parameters, float targetHeading, float maxOutput) {
	_pidgey = pigeonImu;
	_driveTrain = driveTrain;
	_selectedStyle = selectedStyle;
	_targetHeading = targetHeading;
	_servoParams = parameters;
	_maxOutput = maxOutput;
}
ServoHoldHeadingWithImu::ServoHoldHeadingWithImu(CTRE::PigeonImu *pigeonImu, CTRE::Drive::IDrivetrain *driveTrain, CTRE::Drive::Styles::Basic selectedStyle) {
	_pidgey = pigeonImu;
	_driveTrain = driveTrain;
	_selectedStyle = selectedStyle;
}
void ServoHoldHeadingWithImu::Set(CTRE::Drive::Styles::Basic mode, float forward, float strafe){
	Enable(true);
	GoStraight(forward, _targetHeading);
}
float ServoHoldHeadingWithImu::GetImuHeading(){
	double YPR[3];
	_pidgey->GetYawPitchRoll(YPR);
	return YPR[0];
}
void ServoHoldHeadingWithImu::GoStraight(float Y, float targetHeading){
	//We will need some logic to check if we will go straight or not
	if(_enableCompensating == false){
		_isCompensating = false;
	}else if (_pidgey->GetState() != CTRE::PigeonImu::PigeonState::Ready){
		_isCompensating = false;
	}else{
		_isCompensating = true;
	}

	float x_correction;

	if(_isCompensating == false){
		x_correction = 0;
	}else{
		//Let user know we have things to do
		if(_servoParams->P == 0 && _servoParams->I == 0 && _servoParams->D == 0){
			//print out a warning, not sure how to do this in c...
		}

		/* Grab current heading */
		float currentHeading = GetImuHeading();

		/* Grab angualr rate from the pigeon */
		double XYZ_Dps[3];
		_pidgey->GetRawGyro(XYZ_Dps);
		float currentAngularRate = (float)XYZ_Dps[2];

		/* heading PID */
		float headingError = targetHeading - currentHeading;
		float X = (headingError) * _servoParams->P - (currentAngularRate) * _servoParams->D;
		X = CTRE::Utilities::cap(X, _maxOutput);
		x_correction = -X;
	}
	/* select control mode based on selected style*/
	_driveTrain->Set(_selectedStyle, Y, x_correction);
}
void ServoHoldHeadingWithImu::Enable(bool state)
{
	if (state == false){
		_enableCompensating = false;
	}else if(_enableCompensating == true){
		//feature is already enabled, do nothing
	}else{
		//Caller is requesting the servo
		_targetHeading = GetImuHeading();
		_enableCompensating = true;
	}
}
void ServoHoldHeadingWithImu::Disable(){
		Enable(false);
}
}}
