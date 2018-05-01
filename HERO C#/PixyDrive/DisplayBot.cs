/*
 *  Software License Agreement
 *
 * Copyright (C) Cross The Road Electronics.  All rights
 * reserved.
 * 
 * Cross The Road Electronics (CTRE) licenses to you the right to 
 * use, publish, and distribute copies of CRF (Cross The Road) binary firmware files (*.crf) 
 * and software example source ONLY when in use with Cross The Road Electronics hardware products.
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

/****************************************************************************************************/
/***************************************** VERSION **************************************************/
/****************************************************************************************************/
/* Installer:		5.4.1.0
 * CTRE.dll:		5.1.0.0
 * HERO:			1.1.0.0
 * Talon SRX:		11.8.0
 * Pigeon IMU:		0.41
 */

///**
// * Example using both the Pixy Camera and Display Module to demonstate how PixyCamera can be used to follow either
// *  single colored or color coded objects. This example works best with 2 robots which we will call Pixybot and DisplayBot.
// *  
// *  CONFIGURATION SELECTION
// *  To flash the DisplayBot with the DisplayBot configuration, use the drop down menu in the standard toolbar and select CTRE_DisplayBot
// *  Then deploy solution
// *  
// *  PRESETUP
// *  Instructions on how to assemble the robots, flashing the robots, and configuring the Pixy for PixyDrive can be found 
// *  in the GitHub repo @ ...(Link should be here)
// *  
// *  CONTROLS FOR DISPLAYBOT
// *  DisplayBot operates in mecanum drive by defualt and works out of the box with a logitech F710 controller.
// *  The B button enables backlight in the display module
// *  The A button disables backlight in the display module
// *  The Right joystick controls the turn/twist of the robot
// *  The Left joystick controls the forward/backward/right/left drive of the robot
// *  On the DisplayModule, there should be a splt screen where half of it is Magenta and the other half is Green when enabled
// *
// *  SETUP
// *  Position the Display bot center about 1 to 2 feet away from the Pixybot for the best results with the base example code
// *  Enable the Backlight in the Displaybot
// *  Put Pixybot into Pixy drive mode
// *  Using DisplayBot, Drive and watch the PixyBot follow
// *  PD Gains may need to be adjusted 
// */

#if CTRE_DISPLAYBOT

using System;
using System.Threading;
using Microsoft.SPOT;
using Microsoft.SPOT.Hardware;
using CTRE;
using CTRE.Phoenix.MotorControl.CAN;
using CTRE.Phoenix.MotorControl;
using CTRE.Phoenix.Controller;
using CTRE.Gadgeteer.Module;

namespace Hero_PixyDrive
{
    public class Program
    {
        /** Talons to control based on their position of the robot (Can be changed if I.D.s are different)*/
        static TalonSRX LeftRear = new TalonSRX(2);
        static TalonSRX RightRear = new TalonSRX(3);
        static TalonSRX LeftFront = new TalonSRX(1);
        static TalonSRX RightFront = new TalonSRX(4);
        static TalonSRX[] Talons = { LeftFront, LeftRear, RightFront, RightRear };

		/** Attached gamepad to HERO, tested with Logitech F710 */
		static GameController gamepad = new GameController(CTRE.Phoenix.UsbHostDevice.GetInstance(0), 0);

		/** Battery subsystem that can be pulled from any connected TalonSrx*/
		static Battery _Battery = new Battery(LeftRear);

        /** Construct Display Module by defining port and orientation */
        static DisplayModule LCDDisplay = new DisplayModule(CTRE.HERO.IO.Port8, DisplayModule.OrientationType.Landscape_UpsideDown);

        /** Create LED strip controller */
        static CTRE.Phoenix.LEDStripController _LEDStripController = new CTRE.Phoenix.LEDStripController(CTRE.HERO.IO.Port3);

        /** Create color sequence */
        static ColorSequencer _ColorSequencer = new ColorSequencer();

        /** Singleton instance and entry into while loop that runs the desired program*/
        public static void Main()
        {
            /* Tracking gamepad buttons states for single press captures*/
            bool LastBtn1 = false;
            bool LastBtn3 = false;
			bool LastBtn2 = false;

            /* Forword/Backward Scalor */
            const float kScalarX = 0.50f;
            /* Left/Right Scalor */
            const float kScalarY = 0.50f;
            /* Turning Scalor */
            const float kScalarTwist = 0.30f;
            /* Ramp Rate */
            const float kVoltageRampSec = 0.25f;

            /* Configure Talons to operate in percentage VBus mode, and Ramp Up Voltage*/
            foreach (TalonSRX temp in Talons)
            {
                temp.Set(ControlMode.PercentOutput, 0);
                temp.ConfigOpenloopRamp(kVoltageRampSec);
            }

            /* Clear DisplayModule */
            LCDDisplay.ClearSprites();
            LCDDisplay.BacklightEnable = false;
            /* Fill the screen with the target object (Top half magenta, bottom half green) */
            LCDDisplay.AddRectSprite(DisplayModule.Color.Magenta, 0, 0, 160 / 2, 128);
            LCDDisplay.AddRectSprite(DisplayModule.Color.Green, 160 / 2, 0, 160 / 2, 128);

            int colorDelay = 0;
            int i = 0;
            Boolean On = false;
			Boolean LEDEnable = true;

            while (true)
            {
				/* Keep robot enabled if gamepad is connected and in 'D' mode */
				if (gamepad.GetConnectionStatus() == CTRE.Phoenix.UsbDeviceConnection.Connected)
					CTRE.Phoenix.Watchdog.Feed();

				/* Allow user to enable/disable Display Module backlight
                 * Button 2 (A) Disables the backlight
                 * Button 3 (B) Enables the backlight */
				bool Btn1 = gamepad.GetButton(1);
                bool Btn3 = gamepad.GetButton(3);
                if (Btn1 & !LastBtn1)
                {
                    LCDDisplay.BacklightEnable = false;
                }
                else if (Btn3 & !LastBtn3)
                {
                    LCDDisplay.BacklightEnable = true;
                }
                LastBtn1 = Btn1;
                LastBtn3 = Btn3;

				/* Enable/Disable LED's */
				bool Btn2 = gamepad.GetButton(2);
				if (Btn2 & !LastBtn2)
					LEDEnable = !LEDEnable;
				LastBtn2 = Btn2;

				if (gamepad.GetConnectionStatus() == CTRE.Phoenix.UsbDeviceConnection.Connected)
                {
					if (LEDEnable)
					{
						colorDelay++;
						if (colorDelay >= 15)
						{
							_ColorSequencer.Process();
							/* You can change the sequence in ColorSequencer.cs by ordering the premade colors or creating your own values */
							_LEDStripController.Red = _ColorSequencer.Red;
							_LEDStripController.Blue = _ColorSequencer.Blue;
							_LEDStripController.Grn = _ColorSequencer.Green;
							colorDelay = 0;
						}
					}
					else
					{
						_LEDStripController.Red = 0;
						_LEDStripController.Blue = 0;
						_LEDStripController.Grn = 0;
					}
                }

				if(gamepad.GetConnectionStatus() == CTRE.Phoenix.UsbDeviceConnection.NotConnected || _Battery.IsLow())
                {
                    /* Flash RED ??? */
                    i++;
                    if (i >= 200)
                    {
                        On = !On;
                        i = 0;
                    }
                    /* Decide if strip is white or off */
                    if (On == true)
                    {
                        _LEDStripController.Red = 1;
                        _LEDStripController.Blue = 0;
                        _LEDStripController.Grn = 0;
                    }
                    else if (On == false)
                    {
                        _LEDStripController.Red = 0;
                        _LEDStripController.Blue = 0;
                        _LEDStripController.Grn = 0;
                    }
                }

                _LEDStripController.Process();

                /* Regular mecanum drive that is scaled and Gamepad joysticks have been adjusted */
                float X = gamepad.GetAxis(0);
                /* Invert gamepad so forward is truly forward */
                float Y = -1 * gamepad.GetAxis(1);
                float Twist = gamepad.GetAxis(2);
                MecanumDrive(Y * kScalarY, X * kScalarX, Twist * kScalarTwist);
            }
        }

        /**
         * Mecanum Drive that is inverted on the left side and decreases output when low battery
         * 
         * @param   Forward  Forward/Backward drive of mecanum drive
         * @param   Strafe   Left/Right drive of mecanum drive
         * @param   Twist    Turn left/Right of mecanum drive
         */
        private static void MecanumDrive(float Forward, float Strafe, float Twist)
        {
            float leftFrnt = (Forward + Strafe + Twist); /* left front moves positive for forward, strafe-right, turn-right */
            float leftRear = (Forward - Strafe + Twist); /* left rear moves positive for forward, strafe-left, turn-right   */
            float rghtFrnt = (Forward - Strafe - Twist); /* right front moves positive for forward, strafe-left, turn-left  */
            float rghtRear = (Forward + Strafe - Twist); /* right rear moves positive for forward, strafe-right, turn-left  */

            /* Invert left sided motors */
            leftFrnt *= -1;
            leftRear *= -1;

            /* If battery is lower than 10% scale down output */
            if (_Battery.IsLow())
            {
                leftFrnt *= 0.5f;
                leftRear *= 0.5f;
                rghtFrnt *= 0.5f;
                rghtRear *= 0.5f;
            }

            /* Feed values to Talons */
            RightFront.Set(ControlMode.PercentOutput, rghtFrnt);
            RightRear.Set(ControlMode.PercentOutput, rghtRear);
            LeftFront.Set(ControlMode.PercentOutput,leftFrnt);
            LeftRear.Set(ControlMode.PercentOutput,leftRear);
        }
    }
}
#endif