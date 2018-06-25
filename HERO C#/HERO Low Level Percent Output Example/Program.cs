/**
 * Low level CAN framing example for Talon SRX and Victor SPX (when using non-FRC firmware).
 * In other words, the Phoenix library object classes are NOT USED.  Instead simple CAN transmits 
 * are used.
 * 
 * For customers porting to custom platforms (Linux/Raspberry PI/Arduino/etc...) we recommend
 * the following procedure.
 * - Use HERO LifeBoat / roboRIO to place non-FRC firmware into motor controllers
 * - Disconnect motor from Talon/Victor, or clamp motor to a rigid body so motor can  
 *   free-spin without falling off your table.
 * - Deploy this example to a HERO and use a gamepad to control Talon.
 * - Confirm Talon/Victor LEDs follow gamepad stick (x-axis or first axis) or confirm motor spins.
 * - Disconnect gamepad USB from HERO, and confirm motor controller disables (orange-blink).
 * - Take the simple code in this example and port to your platform.  Then retest this procedure.
 */

/**
 * Software License Agreement
 *
 * Copyright (C) Cross The Road Electronics.  All rights
 * reserved.
 * 
 * Cross The Road Electronics (CTRE) licenses to you the right to 
 * use, publish, and distribute copies of CRF (Cross The Road) firmware files (*.crf) and 
 * Phoenix Examples ONLY when in use with CTR Electronics hardware products (Talon/Victor/etc.)
 * as well as the FRC roboRIO when in use in FRC Competition.
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
using System.Threading;
using CTRE.Phoenix.Controller;
using CTRE.Phoenix;
using System;

namespace Hero_Low_Level_Percent_Output_Example
{
	public class Program
	{
		/** Use a USB gamepad plugged into the HERO */
		GameController _gamepad = new GameController(UsbHostDevice.GetInstance());

		public void RunForever()
		{
			/* Pick your motor controller ID */
			const UInt32 deviceNumber = 23; //must be  0 - 62

			/* This is the frame which is used for percent output. */
			const UInt32 CONTROL = 0x040080;

			while (true)
			{
				/* get gamepad/joystick stick value, which is within [-1,+1] */
				float gamepadValue = _gamepad.GetAxis(0);

				/* converts gamepad value [-1,+1] to [-1023,+1023].
				 * Talon/Victor takes a demand value where 1023 is full, 0 is neutral, 
				 * anything in the middle is partial output. This is how PercentOutput mode works. */
				float outputAsFloat = gamepadValue * 1023;

				/* convert the output [-1023,+1023] from floating point to int.  
				 * Basically throw away the fractional part. */
				int outputAsInt = (int)outputAsFloat;

				/* encode output into bytes */
				byte first_byte = (byte)(outputAsInt >> 0x10);
				byte second_byte = (byte)(outputAsInt >> 0x08);
				byte third_byte = (byte)(outputAsInt);

				/* build the control CANbus frame.  The first three bytes is the demand value, 
				 * which typically is the output value [-1023,+1023] */
				ulong frame = 0;

				/*
				 * This assumes we want PercentOutput mode. See Phoenix netmf repository for full implementation (https://github.com/CrossTheRoadElec).
				 */
				frame |= (UInt64)(first_byte);
				frame |= (UInt64)(second_byte) << 0x08;
				frame |= (UInt64)(third_byte) << 0x10;

				/* transmit the CAN bus frame once per loop, 
				 * if you stop sending this then Talon/Victor will disable (blink orange) all the time.
				 */
				CTRE.Native.CAN.Send(CONTROL | deviceNumber | 0x02040000, frame, 8, 0); /* use 0x01040000 for Victor SPX */

				/* 
				 * CTRE Motor Controllers also need a global enable frame.
				 * We will let HERO send this automatically by calling Feed().
				 * On other platforms you must transmit the (nonFRC) enable frame...
				 *  - ArbID is $401BF
				 *  - first data byte is 0 for disable, 1 for enable, 
				 *  - other bytes are zero,  length = 8;
				 *  - use 29bit ID 
				 *  
				 *  For safety reasons, only send the enable frame if Gamepad is plugged into HERO (D-mode).
				 */
				if (_gamepad.GetConnectionStatus() == UsbDeviceConnection.Connected) /* is gamepad USB connected? */
				{
					Watchdog.Feed();  //watch dog times out after 100 ms
				}

				/* wait 10 milliseconds and do it all over again */
				Thread.Sleep(10);
			}
		}

		/* application start point, create the Program instance and jump to RunForever */
		public static void Main()
		{
			new Program().RunForever();
		}
	}
}
