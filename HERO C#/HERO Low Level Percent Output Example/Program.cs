using System.Threading;

using CTRE.Phoenix.MotorControl.CAN;
using CTRE.Phoenix.Controller;
using CTRE.Phoenix;
using CTRE.Phoenix.MotorControl;
using System;
using Microsoft.SPOT;

namespace Hero_Low_Level_Percent_Output_Example
{
    public class Program
    {
        static RobotApplication _robotApp = new RobotApplication();
        public static void Main()
		{
			while (true)
            {
                _robotApp.Run();
            }
        }
    }
    /**
     * The custom robot application.
     */
    public class RobotApplication
    {
		/** Use a USB gamepad plugged into the HERO */
		GameController _gamepad = new GameController(UsbHostDevice.GetInstance());
		TalonSRX temp = new TalonSRX(23);
        public void Run()
        {
			UInt32 deviceNumber = 23; //must be  0 - 62
			const UInt32 CONTROL_3 = 0x040080; //CONTROL_3 is the frame which is used for percent output
			uint baseArbId = deviceNumber | 0x02040000;

			while (true)
            {				
				int output = (int) (1023 * _gamepad.GetAxis(0)); // converts double value which is between -1 and 1 to int between -1023 to 1023

				/* unpack */
				byte first_byte = (byte)(output >> 0x10);
				byte second_byte = (byte)(output >> 0x08);
				byte third_byte = (byte)(output);

				/*
				 * More complex control using demands other than demand0
				 * should get a cache of the previous frame and mask the appropriate bits.
				 * Rather than assuming 0.
				 */

				ulong msg = 0; 

				/* shift in */
				msg |= (UInt64)(first_byte);
				msg |= (UInt64)(second_byte) << 0x08;
				msg |= (UInt64)(third_byte) << 0x10;

				/* 
				 * Talon SRX, Victor SPX, etc., are controlled via two frames...
				 * - Enable frame (ArbID $401BF, first data byte is 0 for disable, 1 for enable, other bytes are zero, DLC=8).
				 * - Control frame (ArbID uses CONTROL_3), each device gets a unique control frame.
				 * ... Enable is taken care of by HERO firmware using Watchdog.Feed()
				 * and CTRE.Native.CAN.Send(...) can't send an enable frame
				 * All CAN Bus framing uses 29bit arbIDs.
				 */

				/* put message on the bus */
				CTRE.Native.CAN.Send(CONTROL_3 | baseArbId, msg, 8, 0);

				Watchdog.Feed();  //watch dog times out after 100 ms

				Thread.Sleep(10);
            }
        }
    }
}
