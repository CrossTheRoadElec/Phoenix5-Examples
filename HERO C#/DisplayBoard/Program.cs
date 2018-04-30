/**
 *  Software License Agreement
 *
 * Copyright (C) Cross The Road Electronics.  All rights
 * reserved.
 * 
 * Cross The Road Electronics (CTRE) licenses to you the right to 
 * use, publish, and distribute copies of CRF (Cross The Road) firmware files (*.crf) and Software
 * API Libraries ONLY when in use with Cross The Road Electronics hardware products.
 * 
 * THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT
 * WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT
 * LIMITATION, ANY WARRANTY OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * CROSS THE ROAD ELECTRONICS BE LIABLE FOR ANY INCIDENTAL, SPECIAL, 
 * INDIRECT OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF
 * PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
 * BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE
 * THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER
 * SIMILAR COSTS, WHETHER ASSERTED ON THE BASIS OF CONTRACT, TORT
 * (INCLUDING NEGLIGENCE), BREACH OF WARRANTY, OR OTHERWISE
 */

/**
 * Example that uses the Driver Module to control a RGB LED strip by using low-side outputs.
 * This example can be setup with the instructions found on the GitHub repository.
 * 
 * If a Talon SRX and a Pigeon IMU are included within your build, ensure your Talon SRX has been flashed 
 * with the included firmware. By default, code  both your Pigeon IMU and Talon SRX I.D. has been set to 0. 
 * 
 * DisplayBoard has two operation modes; Pigeon and Controller. 
 * 
 * Pigeon Mode allows you to use the Pigeon IMU to control the LED strip by tilting the board (Pitch and Roll). 
 * The color from tilting represents the color you would find from a surface of a HSV cylindrical 3D model.
 * 
 * Controller Mode allows you to control the LED strip with the left and right joystick.
 * The left joystick allows you to  control the hue.
 * The right joystick allows you to brightness whenever the left joystick is in use.
 * When both joysticks are idle (centered), DisplayBoard will begin cycling through the color sequence found
 * in ColorSequencer.cs
 * You can modify the brightness of the color cycling by modifying the brightness while using the two joysticks
 * 1. Use the left joystick to exit color cycling.
 * 2. Use the right joystick at the same time to modify brightness.
 * 3. Once the desired brightness has been found, hold the right joystick and let go of the left joystick.
 * 4. Your color cycling will now operate at the brightness choosen.
 * 
 * The values of the HSV are explained here, https://en.wikipedia.org/wiki/HSL_and_HSV
 */

using System;
using System.Threading;
using Microsoft.SPOT;
using CTRE.Gadgeteer.Module;

namespace Hero_DisplayBoard
{
    public class Program
    {
        /* DisplayBoard operation modes */
        private enum States
        {
            Controller,
            Pigeon
        }
        static States OperationState;

        /** Create LED strip controller */
        static CTRE.Phoenix.LEDStripController _LEDStripController = new CTRE.Phoenix.LEDStripController(CTRE.HERO.IO.Port3);

        /** Create a Pigeon IMU for Yaw, Pitch, and Roll (Pigeon over CAN defined with I.D.) */
        static CTRE.Phoenix.Sensors.PigeonIMU _Pigeon = new CTRE.Phoenix.Sensors.PigeonIMU(0);

        /** Create a Talon SRX for controlling the Driver Module (Talon defined with I.D.) */
        static CTRE.Phoenix.MotorControl.CAN.TalonSRX _Talon = new CTRE.Phoenix.MotorControl.CAN.TalonSRX(1);
        static CTRE.Phoenix.MotorControl.CAN.VictorSPX _victor = new CTRE.Phoenix.MotorControl.CAN.VictorSPX(2);
        /** TalonID parameter for CommandLedStrip_Talon() */
        static int _TalonID = (int)_Talon.GetDeviceID();

        /** Create gamepad */
        static CTRE.Phoenix.Controller.GameController _Gamepad = new CTRE.Phoenix.Controller.GameController(CTRE.Phoenix.UsbHostDevice.GetInstance(1), 0);

        /** Create color sequence */
        static ColorSequencer _ColorSequencer = new ColorSequencer();

        /** 2018 Additions */

        /** Display Module, elements, and fonts*/
        static DisplayModule _DisplayModule = new DisplayModule(CTRE.HERO.IO.Port8, DisplayModule.OrientationType.Landscape);
        static DisplayModule.ResourceImageSprite _leftCrossHair, _rightCrossHair;
        static DisplayModule.LabelSprite _labelTitle, _labelRow1, _labelRow2, _labelRow3;

        /* Display Module fonts */
        static Font _smallFont = Properties.Resources.GetFont(Properties.Resources.FontResources.small);
        static Font _bigFont = Properties.Resources.GetFont(Properties.Resources.FontResources.NinaB);

        /** Display Module gauges */
        static VerticalGauge _leftY, _rightY;
        static HorizGauge _leftX, _rightX;

        static CTRE.Phoenix.CANifier _Canifier = new CTRE.Phoenix.CANifier(0);

        static Battery _Battery = new Battery(_Talon);

        public enum driveState : uint
        {
            MotionMagicTalon = 1,
            MotionMagicVictor = 2,
            PercentOutputBoth = 3,
            SensorReset = 4,
        };

        /* Constants */
        static int kTimeout = 10;
        static int kSlotIdx = 0;

        /* PID Constants */
        static float kF = 0.1f;
        static float kP = 0.5f;
        static float kI = 0.005f;
        static float kD = 5.0f;
        static int kIZone = 120;
        static float kNominalOuput = 0;
        static float kPeakOutput = 1;
        static int kMotionVelocity = 10000;
        static int kMotionAcceleration = 5000;

        public static void Main()
        {
            /* Initialize Display elements */
            InitDisplayModule();

            /* Initialize Motion Magic for Talons and Victors */
            InitMotors();

            /* LED Variables */
            float _Brightness = 0.25f;   /* Default LED Brightness */
            bool On = true;             /* Color Flashing state */
            byte i = 0;                 /* Color Duration track variable for flashing */
            int colorDelay = 0;         /* Color Duration track variable for Sequence speed */

            /* Buttons and boolean for MotionMagic Control */
            driveState MotionState = driveState.MotionMagicTalon;
            Boolean lastButton1 = false;
            Boolean lastButton2 = false;
            Boolean lastButton3 = false;
            Boolean lastbutton4 = false;
            while (true)
            {
                if (_Gamepad.GetConnectionStatus() == CTRE.Phoenix.UsbDeviceConnection.Connected)
                    OperationState = States.Controller;
                else
                    OperationState = States.Pigeon;

                if (OperationState == States.Controller)
                {
                    CTRE.Phoenix.Watchdog.Feed();

                    if (_Battery.IsLow())
                    {
                        _labelTitle.SetText("Low Batt...");
                        _labelTitle.SetColor(DisplayModule.Color.Yellow);
                    }
                    else
                    {
                        _labelTitle.SetText("Controller");
                        _labelTitle.SetColor(DisplayModule.Color.Green);
                    }

                    /* Get gamepad values */
                    float LeftX = _Gamepad.GetAxis(0);
                    float LeftY = _Gamepad.GetAxis(1);
                    float RightY = _Gamepad.GetAxis(5);
                    float RightX = _Gamepad.GetAxis(2);

                    /* Deadband gamepad values */
                    Deadband(ref LeftX);
                    Deadband(ref LeftY);
                    Deadband(ref RightX);
                    Deadband(ref RightY);

                    /* Update Guages */
                    UpdateGauge(_leftX, LeftX);
                    UpdateGauge(_leftY, LeftY);
                    UpdateGauge(_rightX, RightX);
                    UpdateGauge(_rightY, RightY);

                    /* Update Crosshairs */
                    _leftCrossHair.SetPosition((int)(30 + 15 * LeftX), 100 + (int)(15 * LeftY));
                    _rightCrossHair.SetPosition((int)(100 + 15 * RightX), 100 + (int)(15 * RightY));

                    /* Get single button presses to control MotionState 8/ */
                    Boolean Button1 = _Gamepad.GetButton(1);
                    Boolean Button2 = _Gamepad.GetButton(2);
                    Boolean Button3 = _Gamepad.GetButton(3);
                    Boolean Button4 = _Gamepad.GetButton(4);
                    if (Button1 && !lastButton1)
                        MotionState = driveState.MotionMagicTalon;
                    else if (Button2 && !lastButton2)
                        MotionState = driveState.PercentOutputBoth;
                    else if (Button3 && !lastButton3)
                        MotionState = driveState.MotionMagicVictor;
                    else if (Button4 && !lastbutton4)
                        MotionState = driveState.SensorReset;
                    lastButton1 = Button1;
                    lastButton2 = Button2;                    lastButton3 = Button3;
                    lastbutton4 = Button4;

                    /* Controls Motoroutput based on MotionState */
                    MotorDriveControl(-LeftY, MotionState);


                    if (LeftX != 0 || LeftY != 0)
                    {
                        /** Left joystick in use, stop color cycling and give user control */

                        /* Grab brightness from the right joystick ([-1,1] + 1 * 0.5 => [0,1]) */
                        float Brightness = (RightY - 1f)  * -0.5f;
                        _Brightness = Brightness;

                        /* Update LED strip with left joystick, right joystick, and brightness */
                        UpdateLedStrip(Brightness, LeftX, LeftY);
                    }
                    else
                    {
                        /** Left joystick not in use, start color cycling */

                        /* You can change the sequence in ColorSequencer.cs by ordering the premade colors or creating your own values */
                        colorDelay++;
                        if(colorDelay >= 3)
                        {
                            _ColorSequencer.Process();
                            colorDelay = 0;
                        }

                        /* Go through a color sequence at half brightness when idle */
                        UpdateLedStrip(_Brightness, _ColorSequencer.Red, _ColorSequencer.Green, _ColorSequencer.Blue);
                    }
                }
                else if (OperationState == States.Pigeon)
                {
                    if (_Battery.IsLow())
                    {
                        _labelTitle.SetText("Low Batt...");
                        _labelTitle.SetColor(DisplayModule.Color.Yellow);
                    }
                    else
                    {
                        _labelTitle.SetText("Pigeon");
                        _labelTitle.SetColor(DisplayModule.Color.Red);
                    }

                    /* Check status of Pigeon to see if it is connected */
                    CTRE.Phoenix.Sensors.PigeonState _PigeonState = _Pigeon.GetState();
                    if (_PigeonState == CTRE.Phoenix.Sensors.PigeonState.Ready)
                    {
                        /** Pigeon connected, giver user tilt control */

                        /* Pull Yaw, Pitch, and Roll from Pigeon */
                        float[] YPR = new float[3];
                        _Pigeon.GetYawPitchRoll(YPR);
                        float Yaw = YPR[0] * 0.1f;
                        float Pitch = YPR[1] * 0.1f;
                        float Roll = YPR[2] * 0.1f;

                        /* Mulitply Pitch and Roll by PI and divide by 180 to get radians for trig functions */
                        float CPitch = Pitch * (float)System.Math.PI / 180;
                        float CRoll = Roll * (float)System.Math.PI / 180;
                        /* Find sine of Pitch and Roll */
                        CPitch = (float)System.Math.Sin(CPitch);
                        CRoll = (float)System.Math.Sin(CRoll);
                        /* Calculate inverse tangent of Pitch and Roll */
                        float Value = (float)System.Math.Atan2(CPitch, CRoll);
                        /* Convert back into degrees */
                        Value = Value * (float)(180 / System.Math.PI);

                        /* Limit the value */
                        if (Value < 0)
                            Value += 360;

                        /* Update LED strip */
                        UpdateLedStrip_Pigeon(Value);
                    }
                    else
                    {
                        /* Pigeon is not Ready/Available, so flash us */
                        i++;
                        if (i >= 100)
                        {
                            On = !On;
                            i = 0;
                        }
                        /* Decide if strip is white or off */
                        if (On == true)
                            /* White */
                            UpdateLedStrip(1, 255, 255, 255);
                        else if (On == false)
                            /* Off */
                            UpdateLedStrip(1, 0, 0, 0);
                    }
                }

                int idx = GetFirstButton(_Gamepad);
                if (idx < 0)
                {
                    _labelRow1.SetColor((DisplayModule.Color)0xA0A0A0); // gray RGB
                    _labelRow2.SetColor((DisplayModule.Color)0xA0A0A0); // gray RGB
                    _labelRow3.SetColor((DisplayModule.Color)0xA0A0A0); // gray RGB

                    float[] ypr = new float[3];
                    _Pigeon.GetYawPitchRoll(ypr);
                    _labelRow1.SetText("Yaw:" + ypr[0]);
                    _labelRow2.SetText("Pitch:" + ypr[1]);
                    _labelRow3.SetText("Roll:" + ypr[2]);
                }
                else
                {
                    switch (idx % 4)
                    {
                        case 0: _labelRow1.SetColor(DisplayModule.Color.Cyan); break;
                        case 1: _labelRow1.SetColor(DisplayModule.Color.Green); break;
                        case 2: _labelRow1.SetColor(DisplayModule.Color.Red); break;
                        case 3: _labelRow1.SetColor(DisplayModule.Color.Yellow); break;

                    }
                    _labelRow1.SetText("Pressed Button " + idx);
                    _labelRow2.SetText("");
                    _labelRow3.SetText("");
                }

                /* All The program to sleep for a little */
                Thread.Sleep(5);
            }
        }

        /** Initialize Display Module elements */
        static void InitDisplayModule()
        {
            _leftY = new VerticalGauge(_DisplayModule, 5, 5, 30, 10, DisplayModule.Color.Cyan, DisplayModule.Color.Blue);
            _rightY = new VerticalGauge(_DisplayModule, 135, 5, 30, 10, DisplayModule.Color.Yellow, DisplayModule.Color.Red);


            _leftX = new HorizGauge(_DisplayModule, 35, 30, 10, 30, DisplayModule.Color.Green, DisplayModule.Color.Magenta);
            _rightX = new HorizGauge(_DisplayModule, 85, 30, 10, 30, DisplayModule.Color.Blue, DisplayModule.Color.Orange);

            _leftCrossHair = _DisplayModule.AddResourceImageSprite(
                                                           Hero_DisplayBoard.Properties.Resources.ResourceManager,
                                                           Hero_DisplayBoard.Properties.Resources.BinaryResources.ch2,
                                                           Bitmap.BitmapImageType.Jpeg,
                                                           30, 100);

            _rightCrossHair = _DisplayModule.AddResourceImageSprite(
                                                           Hero_DisplayBoard.Properties.Resources.ResourceManager,
                                                           Hero_DisplayBoard.Properties.Resources.BinaryResources.ch2,
                                                           Bitmap.BitmapImageType.Jpeg,
                                                           100, 100);

            _labelTitle = _DisplayModule.AddLabelSprite(_bigFont, DisplayModule.Color.White, 40, 0, 80, 16);

            _labelRow1 = _DisplayModule.AddLabelSprite(_smallFont, DisplayModule.Color.White, 30, 46, 100, 15);
            _labelRow2 = _DisplayModule.AddLabelSprite(_smallFont, DisplayModule.Color.White, 30, 58, 100, 15);
            _labelRow3 = _DisplayModule.AddLabelSprite(_smallFont, DisplayModule.Color.White, 30, 70, 100, 15);
        }

        /* Initialize Motors */
       static void InitMotors()
        {
            /* Select Sensor */
            _Talon.ConfigSelectedFeedbackSensor(CTRE.Phoenix.MotorControl.FeedbackDevice.QuadEncoder, 0, kTimeout);
            _victor.ConfigRemoteFeedbackFilter(_Talon.GetDeviceID(), CTRE.Phoenix.MotorControl.RemoteSensorSource.RemoteSensorSource_TalonSRX_SelectedSensor, 0, kTimeout);
            _victor.ConfigSelectedFeedbackSensor(CTRE.Phoenix.MotorControl.RemoteFeedbackDevice.RemoteFeedbackDevice_RemoteSensor0, 0, kTimeout);
            _Talon.SetInverted(true);
            _victor.SetInverted(true);
            _Talon.SetSensorPhase(true);
            _victor.SetSensorPhase(true);

            /* Select Neutral Mode */
            _Talon.SetNeutralMode(CTRE.Phoenix.MotorControl.NeutralMode.Brake);
            _victor.SetNeutralMode(CTRE.Phoenix.MotorControl.NeutralMode.Brake);

            /* Closed loop / Motion Magic Parameters */
            _Talon.Config_kF(kSlotIdx, kF, kTimeout);
            _Talon.Config_kP(kSlotIdx, kP, kTimeout);
            _Talon.Config_kI(kSlotIdx, kI, kTimeout);
            _Talon.Config_kD(kSlotIdx, kD, kTimeout);
            _Talon.Config_IntegralZone(kSlotIdx, kIZone, kTimeout);
            _Talon.SelectProfileSlot(kSlotIdx, 0);
            _Talon.ConfigNominalOutputForward(kNominalOuput, kTimeout);
            _Talon.ConfigNominalOutputReverse(-(kNominalOuput), kTimeout);
            _Talon.ConfigPeakOutputForward(kPeakOutput, kTimeout);
            _Talon.ConfigPeakOutputReverse(-(kPeakOutput), kTimeout);
            _Talon.ConfigMotionCruiseVelocity(kMotionVelocity, kTimeout);
            _Talon.ConfigMotionAcceleration(kMotionAcceleration, kTimeout);

            _Talon.SetStatusFramePeriod(CTRE.Phoenix.MotorControl.StatusFrameEnhanced.Status_10_Targets, 10, kTimeout);
            _Talon.SetStatusFramePeriod(CTRE.Phoenix.MotorControl.StatusFrameEnhanced.Status_2_Feedback0, 10, kTimeout);


            /* Closed loop / Motion Magic Parameters */
            _victor.Config_kF(kSlotIdx, kF, kTimeout);
            _victor.Config_kP(kSlotIdx, 0.50f, kTimeout);
            _victor.Config_kI(kSlotIdx, .005f, kTimeout);
            _victor.Config_kD(kSlotIdx, 2.5f, kTimeout);
            _victor.Config_IntegralZone(kSlotIdx, 120, kTimeout);
            _victor.SelectProfileSlot(kSlotIdx, 0);
            _victor.ConfigNominalOutputForward(kNominalOuput, kTimeout);
            _victor.ConfigNominalOutputReverse(-(kNominalOuput), kTimeout);
            _victor.ConfigPeakOutputForward(kPeakOutput, kTimeout);
            _victor.ConfigPeakOutputReverse(-(kPeakOutput), kTimeout);
            _victor.ConfigMotionCruiseVelocity(kMotionVelocity, kTimeout);
            _victor.ConfigMotionAcceleration(kMotionAcceleration, kTimeout);

            _victor.SetStatusFramePeriod(CTRE.Phoenix.MotorControl.StatusFrameEnhanced.Status_10_Targets, 10, kTimeout);

            _victor.SetSelectedSensorPosition(0, 0, kTimeout);
            _Talon.SetSelectedSensorPosition(0, 0,  kTimeout);
        }

        /* Does all motor output control based on Arguements */
        static void MotorDriveControl(float value, driveState driveOption)
        {
            float SensorNativeUnits = 4096;
            float ServotoRotation = value * 5 * SensorNativeUnits;
            if (driveOption == driveState.MotionMagicTalon)
            {
                _Talon.Set(CTRE.Phoenix.MotorControl.ControlMode.MotionMagic, ServotoRotation);
                _victor.Set(CTRE.Phoenix.MotorControl.ControlMode.PercentOutput, 0);
            }
            else if (driveOption == driveState.MotionMagicVictor)
            {
                _victor.Set(CTRE.Phoenix.MotorControl.ControlMode.MotionMagic, ServotoRotation);
                _Talon.Set(CTRE.Phoenix.MotorControl.ControlMode.PercentOutput, 0);
            }
            else if (driveOption == driveState.PercentOutputBoth)
            {
                _victor.Set(CTRE.Phoenix.MotorControl.ControlMode.PercentOutput, value);
                _Talon.Set(CTRE.Phoenix.MotorControl.ControlMode.PercentOutput, value);
            }
            else if (driveOption == driveState.SensorReset)
            {
                _victor.Set(CTRE.Phoenix.MotorControl.ControlMode.PercentOutput, 0);
                _Talon.Set(CTRE.Phoenix.MotorControl.ControlMode.PercentOutput, 0);
                _victor.SetSelectedSensorPosition(0, 0, kTimeout);
                _Talon.SetSelectedSensorPosition(0, 0, kTimeout);
            }
        }

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

            /* update CANifier's LED strip */
            _Canifier.SetLEDOutput(G, CTRE.Phoenix.CANifier.LEDChannel.LEDChannelA);
            _Canifier.SetLEDOutput(R, CTRE.Phoenix.CANifier.LEDChannel.LEDChannelB);
            _Canifier.SetLEDOutput(B, CTRE.Phoenix.CANifier.LEDChannel.LEDChannelC);
        }

        static void UpdateLedStrip(float Brightness, float X, float Y)
        {
            /* The values of the HSV are explained here,
             * https://en.wikipedia.org/wiki/HSL_and_HSV */

            /* Square it to get bright quickly */
            Brightness = Brightness * Brightness;

            /* Angle */
            float HueDeg = 0;
            /* Finds the angle of the left Stick for hue */
            if (Y != 0 || X != 0)
            {
                /* Find the inverse tangent of X-axis and Y-axis of left joystick for angle */
                HueDeg = (float)System.Math.Atan2(Y, X) * 180f / (float)System.Math.PI;
                /* Keep the angle positive */
                if (HueDeg < 0)
                {
                    HueDeg += 360.0f;
                }
            }

            /* Find the saturation of HSV based on the X and Y value */
            float Saturation = (float)System.Math.Sqrt(X * X + Y * Y);
            /* Constant the value of HSV */
            float Value = 1.0f;

            /* Output after HSV to RGB conversion */
            float R, G, B;
            /* Convert HSV to RGB */
            CTRE.Phoenix.HsvToRgb.Convert(HueDeg, Saturation, Value, out R, out G, out B);
			R = Clamp((int)(R * 255.0));
			G = Clamp((int)(G * 255.0));
			B = Clamp((int)(B * 255.0));


			/* Modify RGB values based on brightness */
			float Red = R * 1f / 255f * Brightness;
            float Green = G * 1f / 255f * Brightness;
            float Blue = B * 1f / 255f * Brightness;

            /* Update RGB values with current RGB */
            _LEDStripController.Red = Red;
            _LEDStripController.Grn = Green;
            _LEDStripController.Blue = Blue;

            /* Update the LED strip through HERO */
            _LEDStripController.Process();

            /* update CANifier's LED strip */
            _Canifier.SetLEDOutput(Green, CTRE.Phoenix.CANifier.LEDChannel.LEDChannelA);
            _Canifier.SetLEDOutput(Red, CTRE.Phoenix.CANifier.LEDChannel.LEDChannelB);
            _Canifier.SetLEDOutput(Blue, CTRE.Phoenix.CANifier.LEDChannel.LEDChannelC);
        }

        static void UpdateLedStrip_Pigeon(float Hue)
        {
            /* 50% brigtness */
            float Brightness = 0.50f;
            /* Hue provided angle from pigeon */
            float HueDeg = Hue;
            /* Constant saturation */
            float Saturation = 1;
            /* Constant value */
            float Value = 1;
            /* Output after HSV to RGB conversion */
            float R, G, B;
            /* Convert HSV to RGB */
            CTRE.Phoenix.HsvToRgb.Convert(HueDeg, Saturation, Value, out R, out G, out B);
			R = Clamp((int)(R * 255.0));
			G = Clamp((int)(G * 255.0));
			B = Clamp((int)(B * 255.0));

			/* Modify RGB values based on brightness */
			float Red = R * 1f / 255f * Brightness;
            float Green = G * 1f / 255f * Brightness;
            float Blue = B * 1f / 255f * Brightness;

            /* Update RGB values with current RGB */
            _LEDStripController.Red = Red;
            _LEDStripController.Grn = Green;
            _LEDStripController.Blue = Blue;

            /* Update the LED strip through HERO */
            _LEDStripController.Process();

            /* update CANifier's LED strip */
            _Canifier.SetLEDOutput(Green, CTRE.Phoenix.CANifier.LEDChannel.LEDChannelA);
            _Canifier.SetLEDOutput(Red, CTRE.Phoenix.CANifier.LEDChannel.LEDChannelB);
            _Canifier.SetLEDOutput(Blue, CTRE.Phoenix.CANifier.LEDChannel.LEDChannelC);
        }

        /* 10 % Deadband for gamepad values */
        static void Deadband(ref float f)
        {
            if (f < -0.1f)
            {
                /* Do nothing, outside deadband */
            }
            else if (f > +0.1f)
            {
                /* Do nothing, outside deadband */
            }
            else
                /* Within deadband, return 0 */
                f = 0;
        }

        /* Change Position of Horizontal Gauge */
        static public void UpdateGauge(HorizGauge gauge, float axis)
        {
            axis += 1.0f; // [0,2]
            axis *= 0.5f; // [0,1]
            gauge.Value = (int)(axis * gauge.MaxValue);
        }

        /* Change Position of Verticle Gauge */
        static public void UpdateGauge(VerticalGauge gauge, float axis)
        {
            axis += 1.0f; // [0,2]
            axis *= 0.5f; // [0,1]
            gauge.Value = (int)(axis * gauge.MaxValue);
        }

        /** Returns only the first Button pressed despite other buttons' Statuses */
        static public int GetFirstButton(CTRE.Phoenix.Controller.GameController gamepad)
        {
            for (uint i = 0; i < 16; ++i)
            {
                if (gamepad.GetButton(i))
                    return (int)i;
            }
            return -1;
        }

		/** Clamp a value to 0 - 255 */
		private static uint Clamp(int i)
		{
			if (i < 0) return 0;
			if (i > 255) return 255;
			return (uint)i;
		}
	}
}
