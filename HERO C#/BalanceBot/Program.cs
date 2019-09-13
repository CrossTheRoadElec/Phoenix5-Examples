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
 * CTRE.dll:		5.1.1.0
 * HERO:			1.1.0.0
 * Talon SRX:		11.8.0
 * Pigeon IMU:		0.41
 * CANifier			0.42
 */

/** 
* Example that uses the Pigeon IMU and Magnetic Encoders to assist the two wheeled robotic system balance.
* The ability to balance relies on the on the cascaded control loop that we have implemented, where the outer
* loop, Velocity, is runnning once every time the inner loop, Balance, excutes 4 times. This allows the outter loop
* to process data. 
* 
* Controls
* Button 1 (X-Button) - Toggle enable/disable the Balance Closed Loop
* Button 2 (A-Button) - Tares the pitch reading of the Balance Bot to 0 at current position (Also resets Trim)
* Button 3 (B-Button) - Cycles between the current PID Set's PID Values [P,I,D]
* Button 4 (Y-Button) - cycles between the various PID Sets [Balance, Drive, Velocity]
* Button 5 (Left-Bumper) - Decrease current PID Value by inc/dec value
* Button 6 (Right-Bumper) - Increase current PID Value by inc/dec value
* Button 10 (Start) - Increase the inc/dec value by 10x, resets to 0.001 after 100
* POV 6 (Left-Dpad) - Decrease Trim by 1;
* PoV 2 (Right-Dpad) - Increase Trim by 1;
* 
* Gain Tuning....
* TO BE DOCUMENTED ONCE PROCEDURE IS FOUND
*/



using CTRE;
using System;
using System.Threading;
using Microsoft;
using Microsoft.SPOT;
using BalanceBot.Platform;

namespace BalanceBot
{

	public class ServoParameters
	{
		public float P = 0;
		public float I = 0;
		public float D = 0;
	}

   public class Program
   {
	   /* Constants */
		const float maxOutput = 1;					// Max output is 0, percent output is [-1, 1]
        const float DegToRad = 0.01745329252f;		// Scalor to convert Degrees to Radians
        const float RadToDeg = 57.2957795131f;		// Scalor to convert Radians to Degrees
		const int kTimeoutMs = 30;                    // Timeout of 30ms for sets and configs

		/* PID for straigtness */
		static float KpGain = 0.2f / 10;
		static float KdGain = 0.0004f /10;
		static float KMaxCorrectionRatio = 0.30f;
		static bool straightState = false;
		static float angleToHold = 0;

		/* Important values when using I-Gains of PID Closed loops (Not in use in our example) */
		static float Iaccum = 0;
        static float accummax = 0;
        static float Iaccum_velocity = 0;
        static float accummax_velocity = 0;

        /* Toggle to disable or enable robot */
        static bool lastButton1 = false;
        static bool OperateState = false;
        static bool manualMode = false;

        /* Pigeon Taring */
        static bool lastButton2 = false;
        static float pitchoffset = 0;

        /* PID control */
        static bool lastButton3 = false;
        static bool lastButton4 = false;
        static bool lastButton5 = false;
        static bool lastButton6 = false;
        static bool lastButton10 = false;
        static float PIDValue = 0;
		static float PIDCycle = 0;
        static float inc_dec = 0.001f;

        static ServoParameters BalancePID = new ServoParameters
        {
            /* acute gains */
            P = 0.010f,
            I = 0.000f,
            D = 0.700f,
        };
        static ServoParameters DrivePID = new ServoParameters
        {
            /* normal gains */
            P = 0.009f,
            I = 0.000f,
            D = 0.700f,
        };
        static ServoParameters VelocityPID = new ServoParameters
        {
            /* velocity gains */
            P = 30,
            I = 0.00f,
            D = 2.00f,
        };

        public static void Main()
        {
            /* Initialize Display */
            CTRE.Gadgeteer.Module.DisplayModule.LabelSprite titleDisplay, pitchDisplay, outputDisplay, PID_PDisplay, PID_IDisplay,
                PID_DDisplay, PIDScalerDisplay, PIDSelectDisplay, batteryDisplay, trimDisplay;

            /* State and battery display in the 1st row */
            titleDisplay = Hardware.Display.AddLabelSprite(Hardware.bigFont, CTRE.Gadgeteer.Module.DisplayModule.Color.Red, 1, 1, 80, 15);
            batteryDisplay = Hardware.Display.AddLabelSprite(Hardware.bigFont, CTRE.Gadgeteer.Module.DisplayModule.Color.Green, 81, 1, 79, 15);

            /* Pitch and output display in the 2nd row */
            pitchDisplay = Hardware.Display.AddLabelSprite(Hardware.bigFont, CTRE.Gadgeteer.Module.DisplayModule.Color.Cyan, 1, 21, 80, 15);
			outputDisplay = Hardware.Display.AddLabelSprite(Hardware.bigFont, CTRE.Gadgeteer.Module.DisplayModule.Color.Cyan, 81, 21, 79, 15);

			/* PID Scalar and angle Trim display in the 3rd row */
			PIDScalerDisplay = Hardware.Display.AddLabelSprite(Hardware.bigFont, CTRE.Gadgeteer.Module.DisplayModule.Color.Yellow, 1, 41, 80, 15);
			trimDisplay = Hardware.Display.AddLabelSprite(Hardware.bigFont, CTRE.Gadgeteer.Module.DisplayModule.Color.Blue, 81, 41, 79, 15);

			/* Gain Display at the bottom */
			PID_PDisplay = Hardware.Display.AddLabelSprite(Hardware.bigFont, CTRE.Gadgeteer.Module.DisplayModule.Color.White, 1, 61, 90, 15);
            PID_IDisplay = Hardware.Display.AddLabelSprite(Hardware.bigFont, CTRE.Gadgeteer.Module.DisplayModule.Color.White, 1, 81, 90, 15);
            PID_DDisplay = Hardware.Display.AddLabelSprite(Hardware.bigFont, CTRE.Gadgeteer.Module.DisplayModule.Color.White, 1, 101, 90, 15);
			PIDSelectDisplay = Hardware.Display.AddLabelSprite(Hardware.bigFont, CTRE.Gadgeteer.Module.DisplayModule.Color.Orange, 91, 81, 60, 15);

			foreach(CTRE.Phoenix.MotorControl.CAN.TalonSRX Talon in Hardware.allTalons)
			{
				/* Factory Default all hardware to prevent unexpected behaviour */
				Talon.ConfigFactoryDefault();

				/* Voltage Compensation on both Talons */
				Talon.ConfigVoltageCompSaturation(10.0f, kTimeoutMs);
				Talon.EnableVoltageCompensation(true);
				Talon.ConfigVoltageMeasurementFilter(32, kTimeoutMs);

				Talon.ConfigNominalOutputForward(0, kTimeoutMs);
				Talon.ConfigNominalOutputReverse(0, kTimeoutMs);
				Talon.ConfigPeakOutputForward(1, kTimeoutMs);
				Talon.ConfigPeakOutputReverse(-1, kTimeoutMs);

				/* Current limiting on both Talons */
				Talon.ConfigContinuousCurrentLimit(15, kTimeoutMs);  // Configured to desired amperage of current draw
				Talon.ConfigPeakCurrentLimit(15, kTimeoutMs);        // Peak current limit set to 0, current limit when current has excedded continout current limit value
				Talon.ConfigPeakCurrentDuration(0, kTimeoutMs);      // Current limit the moment peak current limit has been met by current limit
				Talon.EnableCurrentLimit(true);                    // Enable current limiting

				/* Change Velocity measurement paramters */
				Talon.ConfigVelocityMeasurementPeriod(CTRE.Phoenix.MotorControl.VelocityMeasPeriod.Period_10Ms, kTimeoutMs);
				Talon.ConfigVelocityMeasurementWindow(32, kTimeoutMs);

				/* Speed up Feedback status frame of both Talons */
				Talon.SetStatusFramePeriod(CTRE.Phoenix.MotorControl.StatusFrame.Status_2_Feedback0_, 10, kTimeoutMs);

				/* Speed up Status Frame 4, which provides information about battery */
				Talon.SetStatusFramePeriod(CTRE.Phoenix.MotorControl.StatusFrameEnhanced.Status_4_AinTempVbat, 10, kTimeoutMs);
			}

			/* Speed up Pigeon CAN Frames that are important for the cascade PID loop to operate properly */
			Hardware.pidgey.SetStatusFramePeriod(CTRE.Phoenix.Sensors.PigeonIMU_StatusFrame.BiasedStatus_2_Gyro, 5, kTimeoutMs);
			Hardware.pidgey.SetStatusFramePeriod(CTRE.Phoenix.Sensors.PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR, 5, kTimeoutMs);

			/* Locals used when Gain Scheduling within Balance loop (Inner Loop) */
			float tempP = 0;
            float tempI = 0;
            float tempD = 0;
			ServoParameters currentPID = new ServoParameters();

			float[] XYZ_Dps = new float[3];
			Boolean lowBattery = false;

			CTRE.Phoenix.Controller.GameControllerValues gamepadValues = new CTRE.Phoenix.Controller.GameControllerValues();
			int lastGamepadPOV = 0;
			float angleTrim = 0;

			while (true)
			{
				/* Check to see if gamepad is connected to enable watchdog (Motor Safety) */
				if (Hardware.Gamepad.GetConnectionStatus() == CTRE.Phoenix.UsbDeviceConnection.Connected)
					CTRE.Phoenix.Watchdog.Feed();

				/* Pull values from gamepad */
				float stick = -1 * Hardware.Gamepad.GetAxis(1);
				float turn = Hardware.Gamepad.GetAxis(2);
				CTRE.Phoenix.Util.Deadband(ref stick);          //Deadband
				CTRE.Phoenix.Util.Deadband(ref turn);           //Deadband
				turn *= 0.50f;                                  //Scale turn speed

				Hardware.Gamepad.GetAllValues(ref gamepadValues);
				if (gamepadValues.pov == 2 && lastGamepadPOV != 2)
					angleTrim++;
				else if (gamepadValues.pov == 6 && lastGamepadPOV != 6)
					angleTrim--;
				lastGamepadPOV = gamepadValues.pov;
				trimDisplay.SetText("Trm: " + angleTrim);

				/* Change operation state when Button 1 (X-Button) is pressed */
				bool button1 = Hardware.Gamepad.GetButton(1);
				if (button1 && !lastButton1)
				{
					/* Toggle between operation state and clear accumulated values */
					OperateState = !OperateState;
					Iaccum = 0;
					Iaccum_velocity = 0;
				}
				lastButton1 = button1;

				/* Offset pitch when Button 2 (A-Button) is pressed */
				bool button2 = Hardware.Gamepad.GetButton(2);
				if (button2 && !lastButton2)
				{
					/* Update current pitchoffset with new offset */
					pitchoffset = 0;
					pitchoffset = GetPitch();
					angleTrim = 0;
				}
				lastButton2 = button2;

				/* Cycle through current PID values [P, I, D] when Button 3 (B-Button) is pressed */
				bool button3 = Hardware.Gamepad.GetButton(3);
				if (button3 && !lastButton3)
				{
					/* Select gain to control */
					PIDValue++;
					if (PIDValue > 2)
						PIDValue = 0;
				}
				lastButton3 = button3;

				/* Cycle through PID sets [Soft, Hard, Velocity] when Button 4 (Y-Button) is pressed */
				bool button4 = Hardware.Gamepad.GetButton(4);
				if (button4 && !lastButton4)
				{
					PIDCycle++;
					if (PIDCycle > 2)
						PIDCycle = 0;
				}
				lastButton4 = button4;

				/* Increase the DEC/INC Value for PID Gains by 10x, max of 10.000... when Button 10 (Start-Button) is pressed */
				bool button10 = Hardware.Gamepad.GetButton(10);
				if (button10 && !lastButton10)
				{
					/* Increase the increment/decrement value by x10 */
					inc_dec *= 10;
					if (inc_dec >= 100)
						inc_dec = 0.001f;
				}
				lastButton10 = button10;
				PIDScalerDisplay.SetText("" + inc_dec);

				/* Change the highlighted PID value to inform user which PID is in control */
				if (PIDValue == 0)
				{
					PID_PDisplay.SetColor(CTRE.Gadgeteer.Module.DisplayModule.Color.White);
					PID_IDisplay.SetColor(CTRE.Gadgeteer.Module.DisplayModule.Color.Orange);
					PID_DDisplay.SetColor(CTRE.Gadgeteer.Module.DisplayModule.Color.Orange);
				}
				else if (PIDValue == 1)
				{
					PID_PDisplay.SetColor(CTRE.Gadgeteer.Module.DisplayModule.Color.Orange);
					PID_IDisplay.SetColor(CTRE.Gadgeteer.Module.DisplayModule.Color.White);
					PID_DDisplay.SetColor(CTRE.Gadgeteer.Module.DisplayModule.Color.Orange);
				}
				else if (PIDValue == 2)
				{
					PID_PDisplay.SetColor(CTRE.Gadgeteer.Module.DisplayModule.Color.Orange);
					PID_IDisplay.SetColor(CTRE.Gadgeteer.Module.DisplayModule.Color.Orange);
					PID_DDisplay.SetColor(CTRE.Gadgeteer.Module.DisplayModule.Color.White);
				}

				/* Selects PID set to be in control */
				if (PIDCycle == 0)
				{
					currentPID = BalancePID;
					PIDSelectDisplay.SetText("Soft");
				}
				else if (PIDCycle == 1)
				{
					currentPID = DrivePID;
					PIDSelectDisplay.SetText("Hard");
				}
				else if (PIDCycle == 2)
				{
					currentPID = VelocityPID;
					PIDSelectDisplay.SetText("Velocity");
				}
				PIDControl(currentPID);
				PID_PDisplay.SetText("P: " + currentPID.P);
				PID_IDisplay.SetText("I: " + currentPID.I);
				PID_DDisplay.SetText("D: " + currentPID.D);

				///* Output battery to Display Module */
				float vBat = 0;
				vBat = Hardware.leftTalon.GetBusVoltage();
				batteryDisplay.SetText("Bat: " + vBat);

				if (Hardware.battery.IsLow())
					lowBattery = true;
				else
					lowBattery = false;

				/* If Pigeon is connected and operation state is true, enable balance mode */
				if (Hardware.pidgey.GetState() == CTRE.Phoenix.Sensors.PigeonState.Ready && OperateState == true)
                    manualMode = false;
                else
                    manualMode = true;

                /* Velocity PI */
                //=============================================================================================================================================//
                //=============================================================================================================================================//

                /* Get pitch angular rate */
                Hardware.pidgey.GetRawGyro(XYZ_Dps);
                float pitchRate = XYZ_Dps[0];
				float velocityRPM = -(Hardware.leftTalon.GetSelectedSensorVelocity(0) + Hardware.rightTalon.GetSelectedSensorVelocity(0)) * 0.5f * 600 / 4096;
				float velocityDPS = (velocityRPM) * 6;  //RPM converted into DPS

                /* Velocity setpoint pulled from gamepad throttle joystick */
                float velocitySetpoint = -stick * VelocityPID.D;
                /* Compensate for pitch angular rate when finding velocity */
                float wheelVelocity = ((velocityDPS) + pitchRate) /6  * (float)(System.Math.PI * 6.25f) / 12.00f / 60;   //DPS converted into FPS
                float velocityError = velocitySetpoint - wheelVelocity;

                Iaccum_velocity += (velocityError * VelocityPID.I);
                Iaccum_velocity = CTRE.Phoenix.Util.Cap(Iaccum_velocity, accummax_velocity);

                float pValue_vel = velocityError * VelocityPID.P;
                float iValue_vel = Iaccum_velocity;
                float angleSetpoint = pValue_vel + iValue_vel;

                /* Balance PID, call 4 times per outer call (Cascade PID control) */
                //=============================================================================================================================================//
                //=============================================================================================================================================//
                for (int i = 0; i < 4; i++)
                {
                    Hardware.pidgey.GetRawGyro(XYZ_Dps);				//Get Angular rate for pitch
                    float currentAngularRate = XYZ_Dps[0] * 0.001f;     //Scaled down for easier gain control

                    float currentPitch = GetPitch();					//Get Pitch
                    pitchDisplay.SetText("p: " + currentPitch);			//Update Display

                    float targetPitch = angleSetpoint + angleTrim;
                    float pitchError = targetPitch - currentPitch;

                    float deadband = 5.0f;
                    if (currentPitch > (angleTrim - deadband) && currentPitch < (angleTrim + deadband))
                    {
                        /* Gain schedule when within 5 degrees of current angleTrim */
                        tempP = BalancePID.P;
                        tempI = BalancePID.I;
                        tempD = BalancePID.D;
                    }
                    else
                    {
                        tempP = DrivePID.P;
                        tempI = DrivePID.I;
                        tempD = DrivePID.D;
                        Iaccum = 0;
                    }

					Iaccum += pitchError * tempI;
					Iaccum = CTRE.Phoenix.Util.Cap(Iaccum, accummax);

					/* Clear accumulator when within zone */
					if (currentPitch > -0.5 && currentPitch < 0.5)
                        Iaccum = 0;

                    float pValue = (pitchError) * tempP;
                    float iValue = (Iaccum);
                    float dValue = (currentAngularRate) * tempD;
                    float Output = pValue - dValue + iValue;

                    /* Process output */
                    //=============================================================================================================================================//
                    //=============================================================================================================================================//
                    Output = CTRE.Phoenix.Util.Cap(Output, maxOutput);  //cap value to [-1, 1]

                    if (lowBattery)
                    {
                        /* Scale all drivetrain inputs to 25% if battery is low */
                        batteryDisplay.SetColor(CTRE.Gadgeteer.Module.DisplayModule.Color.Red);
                        Output *= 0.25f;
                        stick *= 0.25f;
                        turn *= 0.25f;
                    }

                    if (manualMode == false)
                    {
                        /* In balance mode, use PI -> PID -> Output */
                        DrivetrainSet(Output, turn);
                        titleDisplay.SetText("Enabled");
                        outputDisplay.SetText("Out: " + Output);
                    }
                    else
                    {
                        /* In maual mode/disabled, use joystick -> Output */
                        DrivetrainSet(stick, turn);
                        titleDisplay.SetText("Disabled");
                        outputDisplay.SetText("Out: " + stick);
                    }

                    /* Balance CAN Frame */
                    byte[] frame = new byte[8];
                    frame[0] = (byte)((int)(pValue * 1000) >> 8);
                    frame[1] = (byte)((int)(pValue * 1000) & 0xFF);
                    frame[2] = (byte)((int)(-dValue * 100000) >> 8);
                    frame[3] = (byte)((int)(-dValue * 100000) & 0xFF);
                    frame[4] = (byte)((int)(iValue * 1000) >> 8);
                    frame[5] = (byte)((int)(iValue * 1000) & 0xFF);
                    frame[6] = (byte)((int)(Output * 1000) >> 8);
                    frame[7] = (byte)((int)(Output * 1000) & 0xFF);
                    ulong data = (ulong)BitConverter.ToUInt64(frame, 0);
                    CTRE.Native.CAN.Send(9, data, 8, 0);
                }

                /* Velocity CAN Frame */
                byte[] frame2 = new byte[8];
                frame2[0] = (byte)((int)(wheelVelocity * 1000) >> 8);
                frame2[1] = (byte)((int)(wheelVelocity * 1000) & 0xFF);
                frame2[2] = (byte)((int)(angleSetpoint * 1000) >> 8);
                frame2[3] = (byte)((int)(angleSetpoint * 1000) & 0xFF);
                frame2[4] = (byte)((int)(pitchRate * 100) >> 8);
                frame2[5] = (byte)((int)(pitchRate * 100) & 0xFF);
                frame2[6] = (byte)((int)(velocityDPS * 100) >> 8);
                frame2[7] = (byte)((int)(velocityDPS * 100) & 0xFF);
                ulong data2 = (ulong)BitConverter.ToUInt64(frame2, 0);
                CTRE.Native.CAN.Send(8, data2, 8, 0);

                Thread.Sleep(5);
            }
        }

        /** Set the drivetrain and convert inputs to set voltage */
        public static void DrivetrainSet(float fset, float tset)
        {
			if (tset == 0)
			{
				/* No turn value requested */
				if (straightState == false)
				{
					/* Update yaw to hold for drive straight */
					angleToHold = GetYaw();
					straightState = true;
				}
				else if (straightState == true)
				{
					/* Use the same angle until yaw changes from user */
				}
			}
			else
			{
				/* turn value not 0, drive normally */
				straightState = false;
			}

			/* Get yaw angular rate */
			float[] XYZ_Dps = new float[3];
            Hardware.pidgey.GetRawGyro(XYZ_Dps);
            float yawRate = XYZ_Dps[2];

            /* Straight drive PD */
            float X = (angleToHold - GetYaw()) * KpGain - (yawRate) * KdGain;
            float MaxThrottle = MaxCorrection(fset, KMaxCorrectionRatio);       // Scale correction output based on throttle with at least 1 percent correction enforced
            X = CTRE.Phoenix.Util.Cap(X, MaxThrottle);							// Cap the output of the correction to the output
            X *= -1;                                                            // For our robot, invert the correction

            /* Selects which variable to use for our turn value */
            if (straightState == true)
                Hardware.drivetrain.Set(CTRE.Phoenix.Drive.Styles.BasicStyle.PercentOutput, fset, X);
            else if (straightState == false)
                Hardware.drivetrain.Set(CTRE.Phoenix.Drive.Styles.BasicStyle.PercentOutput, fset, tset);
        }

        /** Return the pitch with offset */
        public static float GetPitch()
        {
			float[] YPR = new float[3];
			Hardware.pidgey.GetYawPitchRoll(YPR);
			return YPR[1] - pitchoffset;
		}

        /** Return the current yaw */
        public static float GetYaw()
        {
            float[] YPR = new float[3];
            Hardware.pidgey.GetYawPitchRoll(YPR);
            return YPR[0];
        }

        /** PID gains setter */
        public static void PIDControl(ServoParameters PIDtoControl)
        {
            bool button5 = Hardware.Gamepad.GetButton(5);
            bool button6 = Hardware.Gamepad.GetButton(6);
            if (button6 && !lastButton6)
            {
                /* Increment PID value */
                if (PIDValue == 0)
                    PIDtoControl.P += inc_dec;
                else if (PIDValue == 1)
                    PIDtoControl.I += inc_dec;
                else if (PIDValue == 2)
                    PIDtoControl.D += inc_dec;
            }
            else if (button5 && !lastButton5)
            {
                /* Decrement PID value */
                if (PIDValue == 0)
                    PIDtoControl.P -= inc_dec;
                else if (PIDValue == 1)
                    PIDtoControl.I -= inc_dec;
                else if (PIDValue == 2)
                    PIDtoControl.D -= inc_dec;
            }
            lastButton5 = button5;
            lastButton6 = button6;

            /* Force PID gains to be positive */
            if (PIDtoControl.P <= 0)
                PIDtoControl.P = 0;
            if (PIDtoControl.I <= 0)
                PIDtoControl.I = 0;
            if (PIDtoControl.D <= 0)
                PIDtoControl.D = 0;
		}

		/* Scales the scalor based on Y  value, which is the Joystick in our case */
		static float MaxCorrection(float Y, float Scalor)
        {
            Y = (float)System.Math.Abs(Y);
            Y *= Scalor;
            if (Y < 0.01)
                return 0.01f;
            return Y;
        }
    }
}
