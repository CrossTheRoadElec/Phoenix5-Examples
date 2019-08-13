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

/****************************************************************************************************/
/***************************************** VERSION **************************************************/
/****************************************************************************************************/
/**
/* Installer:		5.4.1.0
 * CTRE.dll:		5.1.0.0
 * HERO:			1.1.0.0
 * Talon SRX:		11.8.0
 * Victor SPX:		11.8.0
 * Pigeon IMU:		0.41
 * CANifier			0.42
 */

/**
 * Example that uses the Driver Module and CANifier to control a RGB LED strip by using low-side outputs.
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
 * The project also has the ability to demonstrate the motion magic feature of both the Talon and Victor
 * Using the left joystick's Y-axis allows you to control the output of the motorcontrollers in the various states
 * Button 1 (X-Button) - Motion Magic on Talon SRX using direct feedback sensor, joystick selects distance of [-5, 5]
 * Button 2 (A-Button) - Percent Output on both TalonSRX and Victor SPX, joystick controls percent output [-1, 1]
 * Button 3 (B-Button) - Motion Magic on Victor SPX using remote feedback sensor, joystick selects distance of [-5, 5]
 * Button 4 (Y-Button) - Percent Output of 0 on both Talon SRX and Victor SPX, Sets distance to 0
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
        /** DisplayBoard LED operation modes */
        private enum LEDState
        {
            Controller,
            Pigeon
        }

		/** DisplayBoard motor 0peration modes */
		public enum DriveState
		{
			MotionMagicTalon = 1,
			MotionMagicVictor = 2,
			PercentOutputBoth = 3,
			SensorReset = 4,
		};

		/** Create LED strip controller */
		static CTRE.Phoenix.LEDStripController _LEDStripController = new CTRE.Phoenix.LEDStripController(CTRE.HERO.IO.Port3);

        /** Create a Pigeon IMU for Yaw, Pitch, and Roll (Pigeon over CAN defined with I.D.) */
        static CTRE.Phoenix.Sensors.PigeonIMU _Pigeon = new CTRE.Phoenix.Sensors.PigeonIMU(0);

        /** Create a Talon SRX and Victor SPX to display remote feedback sensor API */
        static CTRE.Phoenix.MotorControl.CAN.TalonSRX _Talon = new CTRE.Phoenix.MotorControl.CAN.TalonSRX(1);
        static CTRE.Phoenix.MotorControl.CAN.VictorSPX _victor = new CTRE.Phoenix.MotorControl.CAN.VictorSPX(2);

        /** Create a gamepad */
        static CTRE.Phoenix.Controller.GameController _Gamepad = new CTRE.Phoenix.Controller.GameController(CTRE.Phoenix.UsbHostDevice.GetInstance(1), 0);

        /** Create color sequence */
        static ColorSequencer _ColorSequencer = new ColorSequencer();

        /** Display Module and it's sprites*/
        static DisplayModule _DisplayModule = new DisplayModule(CTRE.HERO.IO.Port8, DisplayModule.OrientationType.Landscape);
        static DisplayModule.ResourceImageSprite _leftCrossHair, _rightCrossHair;
        static DisplayModule.LabelSprite _labelTitle, _labelRow1, _labelRow2, _labelRow3;

		/** Display Module gauges */
		static VerticalGauge _leftY, _rightY;
		static HorizGauge _leftX, _rightX;

		/* Display Module fonts */
		static Font _smallFont = Properties.Resources.GetFont(Properties.Resources.FontResources.small);
        static Font _bigFont = Properties.Resources.GetFont(Properties.Resources.FontResources.NinaB);

        static CTRE.Phoenix.CANifier _Canifier = new CTRE.Phoenix.CANifier(0);
        static Battery _Battery = new Battery(_Talon);

        /* Constants */
        static int kTimeoutMs = 30;
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
            /* Initialize Display Module elements */
            InitDisplayModule();

            /* Initialize Talon and Victor with various configurations */
            InitMotors();

            /* LED Variables */
            float _Brightness = 0.25f;	/* Default LED Brightness */
            Boolean On = true;          /* Color Flashing state */
            byte i = 0;                 /* Color Duration track variable for flashing */
            int colorDelay = 0;         /* Color Duration track variable for Sequence speed */

			/* State variables */
			DriveState MotionState = DriveState.MotionMagicTalon;
			LEDState OperationState = LEDState.Pigeon;

			/* Buttons and boolean for motor control */
			Boolean lastButton1 = false;
            Boolean lastButton2 = false;
            Boolean lastButton3 = false;
            Boolean lastbutton4 = false;
            while (true)
            {
				/* Check gamepad connection */
                if (_Gamepad.GetConnectionStatus() == CTRE.Phoenix.UsbDeviceConnection.Connected)
                    OperationState = LEDState.Controller;
                else
                    OperationState = LEDState.Pigeon;

                if (OperationState == LEDState.Controller)
                {
                    CTRE.Phoenix.Watchdog.Feed();

                    if (_Battery.IsLow())
                    {
                        _labelTitle.SetText("Low voltage");
                        _labelTitle.SetColor(DisplayModule.Color.Red);
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
					CTRE.Phoenix.Util.Deadband(ref LeftX);
					CTRE.Phoenix.Util.Deadband(ref LeftY);
					CTRE.Phoenix.Util.Deadband(ref RightX);
					CTRE.Phoenix.Util.Deadband(ref RightY);

                    /* Update Guages */
                    UpdateGauge(_leftX, LeftX);
                    UpdateGauge(_leftY, LeftY);
                    UpdateGauge(_rightX, RightX);
                    UpdateGauge(_rightY, RightY);

                    /* Update Crosshairs */
                    _leftCrossHair.SetPosition((int)(30 + 15 * LeftX), 100 + (int)(15 * LeftY));
                    _rightCrossHair.SetPosition((int)(100 + 15 * RightX), 100 + (int)(15 * RightY));

                    /* Get single button presses to control MotionState */
                    Boolean Button1 = _Gamepad.GetButton(1);
                    Boolean Button2 = _Gamepad.GetButton(2);
                    Boolean Button3 = _Gamepad.GetButton(3);
                    Boolean Button4 = _Gamepad.GetButton(4);
                    if (Button1 && !lastButton1)
                        MotionState = DriveState.MotionMagicTalon;
                    else if (Button2 && !lastButton2)
                        MotionState = DriveState.PercentOutputBoth;
                    else if (Button3 && !lastButton3)
                        MotionState = DriveState.MotionMagicVictor;
                    else if (Button4 && !lastbutton4)
                        MotionState = DriveState.SensorReset;
                    lastButton1 = Button1;
                    lastButton2 = Button2;
					lastButton3 = Button3;
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
                else if (OperationState == LEDState.Pigeon)
                {
                    if (_Battery.IsLow())
                    {
                        _labelTitle.SetText("Low Batt...");
                        _labelTitle.SetColor(DisplayModule.Color.Red);
                    }
                    else
                    {
                        _labelTitle.SetText("Pigeon");
                        _labelTitle.SetColor(DisplayModule.Color.Magenta);
                    }

                    /* Check status of Pigeon to see if it is connected */
                    CTRE.Phoenix.Sensors.PigeonState _PigeonState = _Pigeon.GetState();
                    if (_PigeonState == CTRE.Phoenix.Sensors.PigeonState.Ready)
                    {
                        /** Pigeon connected, giver user tilt control */

                        /* Pull Yaw, Pitch, and Roll from Pigeon */
                        float[] YPR = new float[3];
                        _Pigeon.GetYawPitchRoll(YPR);
                        float Pitch = YPR[1];
                        float Roll = YPR[2];

						CTRE.Phoenix.Util.Cap(Pitch, 90);
						CTRE.Phoenix.Util.Cap(Roll, 90);

						float Brightness = 0.5f;

						/* Update LED strip */
						UpdateLedStrip(Brightness, Pitch / 90, Roll / 90, true);
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
                            UpdateLedStrip(1, 255, 255, 255);	/* White */
						else if (On == false)
                            UpdateLedStrip(1, 0, 0, 0);			/* Off */
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

                /* Let he program to sleep for a little */
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
			/* Factory Default all hardware to prevent unexpected behaviour */
			_Talon.ConfigFactoryDefault();
			_victor.ConfigFactoryDefault();

            /* Select Sensor */
            _Talon.ConfigSelectedFeedbackSensor(CTRE.Phoenix.MotorControl.FeedbackDevice.QuadEncoder, 0, kTimeoutMs);
            _victor.ConfigRemoteFeedbackFilter(_Talon.GetDeviceID(), CTRE.Phoenix.MotorControl.RemoteSensorSource.RemoteSensorSource_TalonSRX_SelectedSensor, 0, kTimeoutMs);
            _victor.ConfigSelectedFeedbackSensor(CTRE.Phoenix.MotorControl.RemoteFeedbackDevice.RemoteFeedbackDevice_RemoteSensor0, 0, kTimeoutMs);
            _Talon.SetInverted(true);
            _victor.SetInverted(true);
            _Talon.SetSensorPhase(true);
            _victor.SetSensorPhase(true);

            /* Select Neutral Mode */
            _Talon.SetNeutralMode(CTRE.Phoenix.MotorControl.NeutralMode.Brake);
            _victor.SetNeutralMode(CTRE.Phoenix.MotorControl.NeutralMode.Brake);

            /* Closed loop / Motion Magic Parameters */
            _Talon.Config_kF(kSlotIdx, kF, kTimeoutMs);
            _Talon.Config_kP(kSlotIdx, kP, kTimeoutMs);
            _Talon.Config_kI(kSlotIdx, kI, kTimeoutMs);
            _Talon.Config_kD(kSlotIdx, kD, kTimeoutMs);
            _Talon.Config_IntegralZone(kSlotIdx, kIZone, kTimeoutMs);
            _Talon.SelectProfileSlot(kSlotIdx, 0);
            _Talon.ConfigNominalOutputForward(kNominalOuput, kTimeoutMs);
            _Talon.ConfigNominalOutputReverse(-(kNominalOuput), kTimeoutMs);
            _Talon.ConfigPeakOutputForward(kPeakOutput, kTimeoutMs);
            _Talon.ConfigPeakOutputReverse(-(kPeakOutput), kTimeoutMs);
            _Talon.ConfigMotionCruiseVelocity(kMotionVelocity, kTimeoutMs);
            _Talon.ConfigMotionAcceleration(kMotionAcceleration, kTimeoutMs);

            _Talon.SetStatusFramePeriod(CTRE.Phoenix.MotorControl.StatusFrameEnhanced.Status_10_Targets, 10, kTimeoutMs);
            _Talon.SetStatusFramePeriod(CTRE.Phoenix.MotorControl.StatusFrameEnhanced.Status_2_Feedback0, 10, kTimeoutMs);


            /* Closed loop / Motion Magic Parameters */
            _victor.Config_kF(kSlotIdx, kF, kTimeoutMs);
            _victor.Config_kP(kSlotIdx, kP, kTimeoutMs);
            _victor.Config_kI(kSlotIdx, kI, kTimeoutMs);
            _victor.Config_kD(kSlotIdx, kD/2, kTimeoutMs);
            _victor.Config_IntegralZone(kSlotIdx, kIZone, kTimeoutMs);
            _victor.SelectProfileSlot(kSlotIdx, 0);
            _victor.ConfigNominalOutputForward(kNominalOuput, kTimeoutMs);
            _victor.ConfigNominalOutputReverse(-(kNominalOuput), kTimeoutMs);
            _victor.ConfigPeakOutputForward(kPeakOutput, kTimeoutMs);
            _victor.ConfigPeakOutputReverse(-(kPeakOutput), kTimeoutMs);
            _victor.ConfigMotionCruiseVelocity(kMotionVelocity, kTimeoutMs);
            _victor.ConfigMotionAcceleration(kMotionAcceleration, kTimeoutMs);

            _victor.SetStatusFramePeriod(CTRE.Phoenix.MotorControl.StatusFrameEnhanced.Status_10_Targets, 10, kTimeoutMs);

			/* Set current position to 0, can be changed with button 4 */
            _victor.SetSelectedSensorPosition(0, 0, kTimeoutMs);
            _Talon.SetSelectedSensorPosition(0, 0,  kTimeoutMs);
        }

        /* Does all motor output control based on Arguements */
        static void MotorDriveControl(float value, DriveState driveOption)
        {
            float SensorNativeUnits = 4096;							// Native units per rotation
            float ServotoRotation = value * 5 * SensorNativeUnits;	// 5 rotations forward and reverse
            if (driveOption == DriveState.MotionMagicTalon)
            {
                _Talon.Set(CTRE.Phoenix.MotorControl.ControlMode.MotionMagic, ServotoRotation);
                _victor.Set(CTRE.Phoenix.MotorControl.ControlMode.PercentOutput, 0);
            }
            else if (driveOption == DriveState.MotionMagicVictor)
            {
                _victor.Set(CTRE.Phoenix.MotorControl.ControlMode.MotionMagic, ServotoRotation);
                _Talon.Set(CTRE.Phoenix.MotorControl.ControlMode.PercentOutput, 0);
            }
            else if (driveOption == DriveState.PercentOutputBoth)
            {
                _victor.Set(CTRE.Phoenix.MotorControl.ControlMode.PercentOutput, value);
                _Talon.Set(CTRE.Phoenix.MotorControl.ControlMode.PercentOutput, value);
            }
            else if (driveOption == DriveState.SensorReset)
            {
                _victor.Set(CTRE.Phoenix.MotorControl.ControlMode.PercentOutput, 0);
                _Talon.Set(CTRE.Phoenix.MotorControl.ControlMode.PercentOutput, 0);
                _victor.SetSelectedSensorPosition(0, 0, kTimeoutMs);
                _Talon.SetSelectedSensorPosition(0, 0, kTimeoutMs);
            }
        }

		/** Directly supply RGB values to both CANifier and Driver Module */
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

		/** Creates RGB values based on the position of joysticks or Pitch and Roll*/
        static void UpdateLedStrip(float Brightness, float X, float Y, Boolean isPigeon = false)
        {
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
                    HueDeg += 360.0f;
            }

			/* Find the saturation of HSV based on the X and Y value */
			float Saturation = (float)System.Math.Sqrt(X * X + Y * Y);

			if (isPigeon)
				Saturation *= 5.0f;

			/* Constant the value of HSV */
			float Value = 1.0f;

			/* Convert HSV to RGB */
			float R, G, B;
			CTRE.Phoenix.HsvToRgb.Convert(HueDeg, Saturation, Value, out R, out G, out B);

			UpdateLedStrip(Brightness, R, G, B);
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
				{
					if (i == 0)
						return 1;
					return (int)i;
				}
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
