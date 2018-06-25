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
// *  To flash the PixyBot with the PixyBot configuration, use the drop down menu in the standard toolbar and select CTRE_PixyBot
// *  Then deploy solution
// *  
// *  PRESETUP
// *  Instructions on how to assemble bots, flashing the bots, and configuring the Pixy for PixyDrive can be found 
// *  in the GitHub repo @ ...(Link should be here)
// *  
// *  CONTROLS FOR PIXYBOT  
// *  PixyBot operates in Mecanum drive by default and works out of the box witha Logitech F710 controller
// *  The X button puts the robot in manual drive mode
// *  The B button puts the robot in PixyDrive mode
// *  The A button cycles the LED color on the Pixy Camera
// *  The Left bumper decreases the range between Pixy Camera and the Object in PixyDrive mode
// *  The Right bumper increases the range between Pixy Camera and the object in PixyDrive mode
// *  The Right joystick controls the turn/twist of the robot
// *  The Left joystick controls the forward/backward/right/left drive of the robot
// *
// *  SETUP
// *  Position the Display bot center about 1 to 2 feet away from the Pixybot for the best results with the base example code
// *  Enable the Backlight in the Displaybot
// *  Put Pixybot into Pixy drive mode
// *  Using DisplayBot, drive and watch the PixyBot follow
// *  PD Gains may need to be adjusted 
// */


#if CTRE_PIXYBOT

using System;
using System.Threading;
using Microsoft.SPOT;
using Microsoft.SPOT.Hardware;
using CTRE;
using CTRE.Phoenix.MotorControl.CAN;
using CTRE.Phoenix.Sensors;
using CTRE.Phoenix.MotorControl;
using CTRE.Phoenix.Controller;
using CTRE.Gadgeteer.Module;

namespace Hero_PixyDrive
{
    public class Program
    {
        /** States for inititaing PixyDrive */
        public enum States
        {
            Init,
            PixyDrive,
            ManualDrive
        }
        static States CurrentState;

        /** Talons to control based on their position of the robot (Can be changed if I.D.s are different)*/
        static TalonSRX LeftRear = new TalonSRX(2);
        static TalonSRX RightRear = new TalonSRX(3);
        static TalonSRX LeftFront = new TalonSRX(1);
        static TalonSRX RightFront = new TalonSRX(4);
        static TalonSRX[] Talons = { LeftFront, LeftRear, RightFront, RightRear };

        /**PigeonIMU for Pixybot which by default is connected to left rear talon (Can be changed to any of the other 4 talons) */
        static PigeonIMU Pidgeot = new PigeonIMU(LeftRear);

        /** Attached gamepad to HERO, tested with Logitech F710 */
        static GameController gamepad = new GameController(CTRE.Phoenix.UsbHostDevice.GetInstance(0), 0);

        /** Battery subsystem that can be pulled from any connected TalonSrx*/
        static Battery _Battery = new Battery(LeftFront);

        /** The two SPI modules that are defined later on*/
        static CTRE.ThirdParty.PixyCamera Pixy = new CTRE.ThirdParty.PixyCamera(CTRE.HERO.IO.Port8, 10000);

        /** Create LED strip controller */
        static CTRE.Phoenix.LEDStripController _LEDStripController = new CTRE.Phoenix.LEDStripController(CTRE.HERO.IO.Port3);

        /** Create color sequence */
        static ColorSequencer _ColorSequencer = new ColorSequencer();

        /** A PixyBlock that will be used to take in the new data from the PixyCamera. Contains all Pixy Object information
         * such as X, Y, Width, Height, Area, Angle, and whether is color coded or not. */
        static CTRE.ThirdParty.PixyBlock CurrentBlock = new CTRE.ThirdParty.PixyBlock();
        static CTRE.ThirdParty.PixyBlock blockToFill = new CTRE.ThirdParty.PixyBlock();

        /** Singleton instance and entry into while loop that runs the desired program*/
        public static void Main()
        {
            /* Tracking gamepad buttons states for single press captures*/
            bool LastBtn1 = false;
            bool LastBtn2 = false;
            bool LastBtn3 = false;
            bool LastBtn5 = false;
            bool LastBtn6 = false;

            /* Forword/Backward Scalor */
            const float kScalarX = 0.50f;
            /* Left/Right Scalor */
            const float kScalarY = 0.50f;
            /* Turning Scalor */
            const float kScalarTwist = 0.30f;
            /* Ramp Rate */
            const float kVoltageRampSec = 0.25f;

            /* Gains for PixyDrive (May have to play with this for a better PixyDrive experience)*/
            float KpGain = 0.002f;                  /*P-Gain for Turning*/
            float KdGain = 0.001f;                  /*D-Gain for Turning*/
            float kTurnCorrectionRatio = 0.3f;      /*Ratio for turning speed*/
            float KpGain1 = 0.0006f;                /*P-Gain for driving*/
            float KdGain1 = 0.0005f;                   /*D-Gain for driving*/
            float kForwardCorrectionRatio = 0.6f;   /*Ratio for driving speed*/

            
            /* nonzero to block the config until success, zero to skip checking */
            const int kTimeoutMs = 30;

            /* Configure Talons to operate in percentage VBus mode, and Ramp Up Voltage*/
            foreach (TalonSRX temp in Talons)
            {
                temp.Set(ControlMode.PercentOutput, 0);
                temp.ConfigOpenloopRamp(kVoltageRampSec, kTimeoutMs);
            }

            /* Initiate the program by starting off in Manaul Drive mode */
            CurrentState = States.ManualDrive;

            /* Variables to hold target position and distance of object, set when PixyDrive is initialized */
            int TargetX = 160;
            int TargetArea = 0;

            /* Number of values to use when averaging data from Block, Set to 1 to remove average */
            int AvgCount = 3;
            /* Tracks the Distance between the PixyCamera and the Object using the pervious Area used in PD Loop */
            int LastErrorArea = 0;
            /* Value used to cycle between 3 colors of the LED on the PixyCamera */
            byte RGB = 0;

            Boolean On = false;
            int i = 0;
            int colorCount = 0;

            while (true)
            {
				/* Keep robot enabled if gamepad is connected and in 'D' mode */
				if (gamepad.GetConnectionStatus() == CTRE.Phoenix.UsbDeviceConnection.Connected)
					CTRE.Phoenix.Watchdog.Feed();

				/* Clear the average of X, Y, and Area */
				int AverageX = 0;
                int AverageY = 0;
                int AverageArea = 0;
                /* Adds up the pervious values of X, Y, and Area */
                for (int j = 0; j < AvgCount; j++)
                {
                    /* Run Pixy process to check for Data, pull Block if there is data */
                    Thread.Sleep(5);
                    Pixy.Process();
                    if (Pixy.GetBlock(blockToFill))
                    {
                        /* Store block from Pixy into Local Block */
                        CurrentBlock = blockToFill;
                        /* Uncomment for block data */
                        Debug.Print("===================================");
                        Debug.Print(blockToFill.ToString());
                    }
                    ///* Pull Block data into variables */
                    AverageX += CurrentBlock.X;         /* Pull X from CurrentBlock */
                    AverageY += CurrentBlock.Y;         /* Pull Y from CurrentBlock */
                    AverageArea += CurrentBlock.Area;   /* Pull Area from CurrentBlock */
                }
                /* Finishes the average process by dividing the sum by the number of values */
                AverageX /= AvgCount;
                AverageY /= AvgCount;
                AverageArea /= AvgCount;

                /* Sync Status of Pixy */
                bool Synced = Pixy.Status.Synced;
                /* Duration since last time Pixy had a good data in ms */
                long TimeSinceLastBlock = Pixy.Status.TimeSinceLastBlock;
                /* Uncomment for Pixy status information */
                //Debug.Print("===================================");
                //Debug.Print(Pixy.Status.ToString());

                /* Get angular rate of the robot from Pigeon IMU */
                float[] XYZ_Dps = new float[3];
                Pidgeot.GetRawGyro(XYZ_Dps);
                float CurrentAngularRate = XYZ_Dps[2];

                /* Allows for increasing or decreasing the distance between PixyCamera and the target object
                 * Button 5 (Left-bumper) decreases the range
                 * Button 6 (Right-bumper) increases the range */
                bool Btn5 = gamepad.GetButton(5);
                bool Btn6 = gamepad.GetButton(6);
                if (Btn5 && !LastBtn5)
                {
                    if (TargetArea < (TargetArea + 350))    /* Minimum Distance found through play */
                        TargetArea += 50;                   /* Step closer to object */

                    /* Else, the object is too close to PixyCam */

                }
                if (Btn6 & !LastBtn6)
                {
                    if (TargetArea > (TargetArea - 350))    /* Maximum Distance found through play */
                        TargetArea -= 50;                   /* Step further from object */

                    /* Else, the object is too far from PixyCam */
                }

                /* Control the LED on the Pixy Camera
                 * Button 2 (A) Cycles the color of the LED between Red, Green, and Blue */
                bool Btn2 = gamepad.GetButton(2);
                if (Btn2 && !LastBtn2)
                {
                    /* Cycle between 3 values by incrementing and wrapping around */
                    if (RGB == 2)
                        RGB = 0;
                    else
                        RGB++;

                    /* Set the states of the color based on RGB value
                     * Colors can be modified by doing different combinations of 0 and 255 */
                    if (RGB == 0)
                        Pixy.SetLED(true, false, false);
                    else if (RGB == 1)
                        Pixy.SetLED(false, true, false);
                    else if (RGB == 2)
                        Pixy.SetLED(false, false, true);
                }
                LastBtn2 = Btn2;

                /* Always give user the ability to enter Manaul Drive mode
                 * Button 1 (X) changes the PixyDrive State to Manual Drive State */
                bool Btn1 = gamepad.GetButton(1);
                if (Btn1 && !LastBtn1)
                    CurrentState = States.ManualDrive;
                LastBtn1 = Btn1;

                /* State machine that determines if Pixy Target needs to be initiated, Pixy object is in target of Pixy Camera parameters, or
                 * robot is in Manual Drive mode */
                switch (CurrentState)
                {
                    case States.Init:
                        /* Grab Inital area and X-position, and set as target values */
                        TargetX = AverageX;
                        TargetArea = AverageArea;
                        CurrentState = States.PixyDrive;
                        break;

                    case States.PixyDrive:
                        /* Update the LED strip through HERO */
                        _LEDStripController.Process();
                        /* Bulk of the PixyDrive, where it uses the current Pixy data to allign with the TargetArea and TargetX */
                        if (Synced == true && TimeSinceLastBlock < 50)  /* Time since last block should be around 50 ms */
                        {
                            /* Object is in View of camera and we getting good data */

                            /* Find the Error between the desired values between the the current values of Area and X */
                            int ErrorX = TargetX - AverageX;
                            int ErrorArea = TargetArea - AverageArea;
                            /* Variable used later on to smooth out PD of Drive */
                            int ErrorArea_Kd1 = ErrorArea - LastErrorArea;
                            /* Track the last error value and compared to next error value */
                            LastErrorArea = ErrorArea;

                            /* Checks to see if the error is within an allowable margin,
                             * ErrorX is good in the sense that there is minimal noise on X and Y
                             * Error Area can be tweeked according to application as larger objects will have more noise and smallear objects have
                             * less noise and will require more preciseness for better response */
                            if ((ErrorX < 3 && ErrorX > -3) && (ErrorArea < 20 && ErrorArea > -20))
                            {
                                /* Consisered centerd and a good distance away so stop */
                                TankDrive(0, 0);

                                /* Update RGB values with current RGB */
                                _LEDStripController.Red = 0;
                                _LEDStripController.Grn = 1;
                                _LEDStripController.Blue = 0;
                            }
                            else
                            {
                                /* There is error with either X or Area, so fix that */
                                bool xOutOfBounds = OutOfBoundsX(AverageX, CurrentBlock.Width);
                                /* If object is out bounds, kill the forward and backward drive, turn right or left to still follow */
                                if (xOutOfBounds)
                                {
                                    /* Set ErrorArea to 0 to prevent forward/backward drive in PixyDrive */
                                    ErrorArea = 0;
                                    ErrorArea_Kd1 = 0;
                                }
                                /* PD Loop for PixyDrive until centered
                                 * Turning uses ErrorX for P and CurrentAngularRate for D
                                 * Forward/Backward uses ErrorArea for P and ErrorArea_Kd1 (ErrorArea - LastErrorArea) for D */
                                float Turn = (ErrorX) * KpGain - (CurrentAngularRate) * KdGain;
                                float Forward = (ErrorArea) * KpGain1 + (ErrorArea_Kd1) * KdGain1;
                                /* Cap the return values to turn down the output of the motors */
                                Turn = Cap(Turn, kTurnCorrectionRatio);
                                Forward = Cap(Forward, kForwardCorrectionRatio);

                                /* With the turn and forward values found, feed that to the TankDrive accordingly */
                                float LeftAuto = ((-Turn + Forward));
                                float RightAuto = ((Turn + Forward));
                                TankDrive(LeftAuto, RightAuto);

                                /* Update RGB values with current RGB */
                                if ((System.Math.Abs(LeftAuto) + (System.Math.Abs(RightAuto)) <= 0.10f))
                                {
                                    colorCount++;
                                    if (colorCount >= 5) {
                                        _LEDStripController.Red = 0;
                                        _LEDStripController.Grn = 1;
                                        _LEDStripController.Blue = 0;
                                    }
                                }
                                else
                                {
                                    colorCount = 0;
                                    _LEDStripController.Red = 0;
                                    _LEDStripController.Grn = 1;
                                    _LEDStripController.Blue = 1;
                                }
                            }
                        }
                        else
                        {
                            /* We are in PixyDrive Mode, but there is no object found, so stop */
                            TankDrive(0, 0);

							_LEDStripController.Red = 0;
							_LEDStripController.Grn = 0;
							_LEDStripController.Blue = 1;
						}
                        break;

                    case States.ManualDrive:

                        /* Update RGB values with current RGB */
                        _LEDStripController.Red = 1;
                        _LEDStripController.Grn = 0;
                        _LEDStripController.Blue = 0.8f;

                        /* Allow the user to enter PixyDrive mode
                         * Button 3 (B) Changes Manual Drive Stat to PixyDrive State */
                        bool Btn3 = gamepad.GetButton(3);
                        if (Btn3 && !LastBtn3)
                            CurrentState = States.Init;
                        LastBtn3 = Btn3;

                        /* Regular mecanum drive that is scaled and Gamepad joysticks have been adjusted */
                        float X = gamepad.GetAxis(0);
                        /* Invert gamepad so forward is truly forward */
                        float Y = -1 * gamepad.GetAxis(1);
                        float Twist = gamepad.GetAxis(2);
                        MecanumDrive(Y * kScalarY, X * kScalarX, Twist * kScalarTwist);
                        break;
                }

				if (gamepad.GetConnectionStatus() == CTRE.Phoenix.UsbDeviceConnection.NotConnected || _Battery.IsLow())
				{
					/* Flash RED ??? */
					i++;
                    if (i >= 100)
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
            }
        }

        /**
         * Check to see if Object has reached the X-axis boundaries
         * 
         * @param   X       PixyCamera X value
         * @param   Width   PixCamera Width Value
         * @return  bool    Whether the object has reached the boundaries or not
         */
        private static bool OutOfBoundsX(int X, int Width)
        {
            /* Constants found using GUI and Block printf */
            const int Left = 40;
            const int Right = 280;

            /* Has object reached boundary? */
            if (X < Left)
                return true;
            if ((X + Width) > Right)
                return true;
            return false;
        }

        /**
         * Check to see if Object has reached the Y-axis boundaries
         * 
         * @param   Y       PixyCamera Y value
         * @param   Height  PixCamera Height Value
         * @return  bool    Whether the object has reached the boundaries or not
         */
        private static bool OutOfBoundsY(int Y, int Height)
        {
            /* Constants found using GUI and Block printf */
            const int Top = 20;
            const int Bottom = 180;

            /* Has object reached boundary? */
            if (Y < Top)
                return true;
            if ((Y + Height) > Bottom)
                return true;
            return false;
        }

        /**
         * Caps inputted value to desired peak if necessary
         * 
         * @param   Value    Value to cap
         * @param   Peak     Peak value to cap at       
         * @return  Value    Capped value
         */
        private static float Cap(float Value, float Peak)
        {
            if (Value < -Peak)
                return -Peak;
            if (Value > +Peak)
                return +Peak;
            return Value;
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
            float leftFrnt = (Forward + Strafe + Twist); // left front moves positive for forward, strafe-right, turn-right
            float leftRear = (Forward - Strafe + Twist); // left rear moves positive for forward, strafe-left, turn-right
            float rghtFrnt = (Forward - Strafe - Twist); // right front moves positive for forward, strafe-left, turn-left
            float rghtRear = (Forward + Strafe - Twist); // right rear moves positive for forward, strafe-right, turn-left

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

            /* Feed Talons */
            RightFront.Set(ControlMode.PercentOutput, rghtFrnt);
            RightRear.Set(ControlMode.PercentOutput, rghtRear);
            LeftFront.Set(ControlMode.PercentOutput, leftFrnt);
            LeftRear.Set(ControlMode.PercentOutput, leftRear);
        }

        /**
         * Tank Drive that is inverted on the left side and decreases output when low battery
         * 
         * Used during PixyDrive
         * 
         * @param   Left    Input for left side of robot, not inverted
         * @param   Right   Input for right side of robot
         */
        static void TankDrive(float Left, float Right)
        {
            /* Invert left sided motors */
            Left *= -1;

            /* If battery is lower than 10% scale down output */
            if (_Battery.IsLow())
            {
                Right *= 0.5f;
                Left *= 0.5f;
            }

            /* Feed Talons */
            RightFront.Set(ControlMode.PercentOutput, Right);
            RightRear.Set(ControlMode.PercentOutput, Right);
            LeftFront.Set(ControlMode.PercentOutput, Left);
            LeftRear.Set(ControlMode.PercentOutput, Left);
        }

        /**
         * Updates the LED strip (Talon/HERO) when given Brightness, R, G, and B
         * 
         * @param   Brightness  Brightness value from 0 to 1, modifies RGB values
         * @param   R           Red value of RGB
         * @param   G           Green value of RGB
         * @param   B           Blue value of RGB
         */
        static void UpdateLedStrip(float Brightness, float R, float G, float B)
        {
            /* Modify RGB values with the given Brightness */
            R *= Brightness;
            G *= Brightness;
            B *= Brightness;

            /* Update RGB values with current RGB */
            _LEDStripController.Red = R;
            _LEDStripController.Grn = G;
            _LEDStripController.Blue = B;

            /* Update the LED strip through HERO */
            _LEDStripController.Process();
        }
    }
}
#endif
