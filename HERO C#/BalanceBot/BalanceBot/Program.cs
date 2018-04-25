using CTRE;
using System;
using System.Threading;
using Microsoft;
using Microsoft.SPOT;
using BalanceBot.Platform;

namespace BalanceBot
{
    public class Program
    {
        /* Constants */
        const float maxOutput = 1;					// Max output is 0, percent output is [-1, 1]
        const float DegToRad = 0.01745329252f;		// Scalor to convert Degrees to Radians
        const float RadToDeg = 57.2957795131f;		// Scalor to convert Radians to Degrees
		const int kTimeout = 10;                    // Timeout of 10ms for sets and configs

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

        static CTRE.Phoenix.ServoParameters BalancePID = new CTRE.Phoenix.ServoParameters
        {
            /* acute gains */
            P = 0.010f,
            I = 0.000f,
            D = 0.700f,
        };
        static CTRE.Phoenix.ServoParameters DrivePID = new CTRE.Phoenix.ServoParameters
        {
            /* normal gains */
            P = 0.009f,
            I = 0.000f,
            D = 0.700f,
        };
        static CTRE.Phoenix.ServoParameters VelocityPID = new CTRE.Phoenix.ServoParameters
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
                PID_DDisplay, PIDScalerDisplay, PIDSelectDisplay, batteryDisplay;

            /* State and battery display */
            titleDisplay = Hardware.Display.AddLabelSprite(Hardware.bigFont, CTRE.Gadgeteer.Module.DisplayModule.Color.Red, 1, 1, 80, 15);
            batteryDisplay = Hardware.Display.AddLabelSprite(Hardware.bigFont, CTRE.Gadgeteer.Module.DisplayModule.Color.Green, 80, 1, 80, 15);

            /* Pitch and output display on top right */
            pitchDisplay = Hardware.Display.AddLabelSprite(Hardware.bigFont, CTRE.Gadgeteer.Module.DisplayModule.Color.Cyan, 1, 21, 80, 15);
            outputDisplay = Hardware.Display.AddLabelSprite(Hardware.bigFont, CTRE.Gadgeteer.Module.DisplayModule.Color.Cyan, 80, 21, 80, 15);

            /* Gain Display at the bottom */
            PIDScalerDisplay = Hardware.Display.AddLabelSprite(Hardware.bigFont, CTRE.Gadgeteer.Module.DisplayModule.Color.Yellow, 1, 41, 80, 15);
			PID_PDisplay = Hardware.Display.AddLabelSprite(Hardware.bigFont, CTRE.Gadgeteer.Module.DisplayModule.Color.White, 1, 61, 90, 15);
            PID_IDisplay = Hardware.Display.AddLabelSprite(Hardware.bigFont, CTRE.Gadgeteer.Module.DisplayModule.Color.White, 1, 81, 90, 15);
            PID_DDisplay = Hardware.Display.AddLabelSprite(Hardware.bigFont, CTRE.Gadgeteer.Module.DisplayModule.Color.White, 1, 101, 90, 15);
			PIDSelectDisplay = Hardware.Display.AddLabelSprite(Hardware.bigFont, CTRE.Gadgeteer.Module.DisplayModule.Color.Orange, 91, 81, 60, 15);


			Hardware.leftTalon.ConfigVoltageCompSaturation(10.0f, kTimeout);
			Hardware.leftTalon.EnableVoltageCompensation(true);
			Hardware.leftTalon.ConfigVoltageMeasurementFilter(32, kTimeout);

			Hardware.leftTalon.ConfigNominalOutputForward(0, kTimeout);
			Hardware.leftTalon.ConfigNominalOutputReverse(0, kTimeout);
			Hardware.leftTalon.ConfigPeakOutputForward(1, kTimeout);
			Hardware.leftTalon.ConfigPeakOutputReverse(-1, kTimeout);


			Hardware.rightTalon.ConfigVoltageCompSaturation(10.0f, kTimeout);
			Hardware.rightTalon.EnableVoltageCompensation(false);
			Hardware.rightTalon.ConfigVoltageMeasurementFilter(32, kTimeout);

			Hardware.rightTalon.ConfigNominalOutputForward(0, kTimeout);
			Hardware.rightTalon.ConfigNominalOutputReverse(0, kTimeout);
			Hardware.rightTalon.ConfigPeakOutputForward(1, kTimeout);
			Hardware.rightTalon.ConfigPeakOutputReverse(-1, kTimeout);


			/* Current limiting on both Talons */
			Hardware.leftTalon.ConfigContinuousCurrentLimit(20, kTimeout);// Configured to desired amperage of current draw
			Hardware.leftTalon.ConfigPeakCurrentLimit(20, kTimeout);       // Peak current limit set to 0, current limit when current has excedded continout current limit value
			Hardware.leftTalon.ConfigPeakCurrentDuration(0, kTimeout);    // Current limit the moment peak current limit has been met by current limit
			Hardware.leftTalon.EnableCurrentLimit(true);        // Enable current limiting

			Hardware.rightTalon.ConfigContinuousCurrentLimit(20, kTimeout);
			Hardware.rightTalon.ConfigPeakCurrentLimit(20, kTimeout);
			Hardware.rightTalon.ConfigPeakCurrentDuration(0, kTimeout);
			Hardware.rightTalon.EnableCurrentLimit(true);

			/* Set sensor phase of gearbox (May have to change phase of right side instead */
			//Hardware.leftGearbox.SetSensorPhase(true);

			/* Change Velocity measurement paramters */
			Hardware.rightTalon.ConfigVelocityMeasurementPeriod(CTRE.Phoenix.MotorControl.VelocityMeasPeriod.Period_10Ms, kTimeout);
			Hardware.rightTalon.ConfigVelocityMeasurementWindow(32, kTimeout);
			Hardware.leftTalon.ConfigVelocityMeasurementPeriod(CTRE.Phoenix.MotorControl.VelocityMeasPeriod.Period_10Ms, kTimeout);
			Hardware.leftTalon.ConfigVelocityMeasurementWindow(32, kTimeout);

			/* Speed up Feedback status frame of both Talons */
			Hardware.leftTalon.SetStatusFramePeriod(CTRE.Phoenix.MotorControl.StatusFrame.Status_2_Feedback0_, 10, kTimeout);
			Hardware.rightTalon.SetStatusFramePeriod(CTRE.Phoenix.MotorControl.StatusFrame.Status_2_Feedback0_, 10, kTimeout);

			/* Speed up Pigeon CAN Frames that are important for the cascade PID loop to operate properly */
			Hardware.pidgey.SetStatusFramePeriod(CTRE.Phoenix.Sensors.PigeonIMU_StatusFrame.BiasedStatus_2_Gyro, 5, kTimeout);
			Hardware.pidgey.SetStatusFramePeriod(CTRE.Phoenix.Sensors.PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR, 5, kTimeout);

			/* Speed up Status Frame 4, which provides information about battery */
			Hardware.leftTalon.SetStatusFramePeriod(CTRE.Phoenix.MotorControl.StatusFrameEnhanced.Status_4_AinTempVbat, 10, kTimeout);
			Hardware.rightTalon.SetStatusFramePeriod(CTRE.Phoenix.MotorControl.StatusFrameEnhanced.Status_4_AinTempVbat, 10, kTimeout);

			/* Locals used when Gain Scheduling within Balance loop (Inner Loop) */
			float tempP = 0;
            float tempI = 0;
            float tempD = 0;
			CTRE.Phoenix.ServoParameters currentPID = new CTRE.Phoenix.ServoParameters();

			float[] XYZ_Dps = new float[3];
			Boolean lowBattery = false;

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

                /* Balance PID, call 4 times per outer call */
                //=============================================================================================================================================//
                //=============================================================================================================================================//
                for (int i = 0; i < 4; i++)
                {
                    Hardware.pidgey.GetRawGyro(XYZ_Dps);
                    float currentAngularRate = XYZ_Dps[0] * 0.001f;       //Scaled down for easier gain control

                    float currentPitch = GetPitch();
                    pitchDisplay.SetText("p: " + currentPitch);

                    float targetPitch = angleSetpoint;
                    float pitchError = targetPitch - currentPitch;

                    Iaccum += pitchError;
                    Iaccum = CTRE.Phoenix.Util.Cap(Iaccum, accummax);

                    float deadband = 5.0f;
                    if (currentPitch > -deadband && currentPitch < deadband)
                    {
                        /* Gain schedule when within 5 degrees  of 0 */
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

                    if (currentPitch > -0.5 && currentPitch < 0.5)
                        Iaccum = 0;     //Clear accumulator when within zone

                    float pValue = (pitchError) * tempP;
                    float iValue = (Iaccum) * tempI;
                    float dValue = (currentAngularRate) * tempD;
                    float Output = pValue - dValue + iValue;

                    /* Process output */
                    //=============================================================================================================================================//
                    //=============================================================================================================================================//
                    Output = CTRE.Phoenix.Util.Cap(Output, maxOutput);  //cap value from [-1, 1]

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
            float MaxThrottle = MaxCorrection(fset, KMaxCorrectionRatio);       // Scale correction output based on throttle
            X = CTRE.Phoenix.Util.Cap(X, MaxThrottle);
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
        public static void PIDControl(CTRE.Phoenix.ServoParameters PIDtoControl)
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