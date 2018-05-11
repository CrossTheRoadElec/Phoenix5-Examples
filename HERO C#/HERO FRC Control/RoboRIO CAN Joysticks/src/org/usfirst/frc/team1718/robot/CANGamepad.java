package org.usfirst.frc.team1718.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.can.CANJNI;

public class CANGamepad {
	
	/* The arbitration ID is used to identify
	 * individual messages on the CAN bus.
	 * CAN in FRC typically uses a 29-bit extended arbitration ID.
	 * 
	 * The binary representation of an arbID is broken down as follows:
	 * 
	 *      DeviceType |   Manufacturer  | API(Class)  API(Index)| Device Number
	 *         5 bits  |      8 bits     |   6 bits       4 bits |   6 bits
	 *       0 0 0 0 0 | 0 0 0 0 0 0 0 0 | 0 0 0 0 0 0   0 0 0 0 | 0 0 0 0 0 0
	 *   
	 * The base Arbitration ID for a given message is the
	 *   arbID without a specified device number.
	 *   
	 * So for a Joystick status frame, we need to determine a base arbID.
	 * 
	 *  Device Type: 30.  This is arbitrary, but it's one less than the maximum.
	 *  				  This is typically the number we encourage teams to use.
	 *  Manufacturer: 8.  This is the manufacturer ID set aside for teams.  You should always use 8,
	 *  					as other manufacuters (like CTRE) have their own value.  Using 
	 *  					a distinct value keeps CAN frames from conflicting.
	 *  API (Class): 5.   The API Class is the "type" of frame we're sending.  You would
	 *  					typically have a different class for status frames, control frames,
	 *  					configuration parameters, etc.  We're only using one frame in this
	 *  					example so it can be whatever we want.
	 *  API (Idx): 0.     If you need multiple of the same type of message (for example, if your
	 *  					device has enough status data to take up two or more messages of the same class),
	 *  					you index the frames within the class.  Fortunately we can fit all the
	 *  					joystick data into one 8-byte frame, so we only need one API Index.
	 *  Device Number:	  This will be set when we send the frame - we'll have one for each
	 *  					joystick we want to send data over the CAN bus.
	 *  
	 *  To determine our base arbId, we would "assemble" it as follows:
	 *   
	 *      DeviceType |   Manufacturer  | API(Class)  API(Index)| Device Number
	 * Dec:     30	   |        8        |     5            0    |       0
	 * Bin:  1 1 1 1 0 | 0 0 0 0 1 0 0 0 | 0 0 0 1 0 1   0 0 0 0 | 0 0 0 0 0 0
	 * Hex:  1|   E    |    0   |   8    |    1   |    4    |    0    |   0
	 * 
	 * baseArbId = 0x1E081400
	 * 
	 */
	final static int baseArbId = 0x1E081400;
	
	
	/* This function sends the joystick data over the CAN bus.
	 * 
	 * It currently supports generic HID controllers (like the Logitech 310)
	 * and XBox controllers.
	 * 
	 * It uses the baseArbId and does a bitwise OR operation with the Gamepad's port number.
	 * This means the port number is used as the device ID, guaranteeing
	 * that every individual gamepad will have its own CAN frame.
	 */
	public static void send(Joystick stick){
		//This will hold the Joystick Data to send via CAN.
		byte[] data = new byte[8];
		
		/* Joystick Axes will be encoded as a value from -127 to +127 */
		data[0] = (byte)(stick.getRawAxis(0) * 127);
		data[1] = (byte)(stick.getRawAxis(1) * 127);
		data[2] = (byte)(stick.getRawAxis(2) * 127);
		data[3] = (byte)(stick.getRawAxis(3) * 127);
		if(stick.getAxisCount() >=6){
			data[4] = (byte)(stick.getRawAxis(4) * 127);
			data[5] = (byte)(stick.getRawAxis(5) * 127);
		}
		else{
			data[4] = 0;
			data[5] = 0;
		}
		
		/* Encode POV as index from 0 to 7 */
		byte b6 = 0;
		int pov = stick.getPOV();
		boolean povPressed = pov >= 0;
		if(povPressed){
			b6 = (byte)(pov / 45);
		}
		else{
			b6 = -1;
		}
		
		/* Encode Button Values as Bits */
		b6 <<= 1;
		b6 |= (byte)(stick.getRawButton(4) ? 1:0);
		b6 <<= 1;
		b6 |= (byte)(stick.getRawButton(3) ? 1:0);
		b6 <<= 1;
		b6 |= (byte)(stick.getRawButton(2) ? 1:0);
		b6 <<= 1;
		b6 |= (byte)(stick.getRawButton(1) ? 1:0);
		data[6] = b6;
		
		byte b7 = 0;
		if(stick.getButtonCount() >= 12){
			b7 |= (byte)(stick.getRawButton(12) ? 1:0);
			b7 <<= 1;
			b7 |= (byte)(stick.getRawButton(11) ? 1:0);
			b7 <<= 1;
		}
		b7 |= (byte)(stick.getRawButton(10) ? 1:0);
		b7 <<= 1;
		b7 |= (byte)(stick.getRawButton(9) ? 1:0);
		b7 <<= 1;
		b7 |= (byte)(stick.getRawButton(8) ? 1:0);
		b7 <<= 1;
		b7 |= (byte)(stick.getRawButton(7) ? 1:0);
		b7 <<= 1;
		b7 |= (byte)(stick.getRawButton(6) ? 1:0);
		b7 <<= 1;
		b7 |= (byte)(stick.getRawButton(5) ? 1:0);
		data[7] = b7;
		
		
		/* Send the CAN Frame.
		 * This uses the HID Port as the deviceID.
		 * 
		 * A zero is passed in for the periodMs so that it's only sent once.
		 * We'll send this frame only on command (instead of periodically)
		 * so that we can check for stale frames on the HERO side.
		 */
		CANJNI.FRCNetCommCANSessionMuxSendMessage(baseArbId | stick.getPort(), data, 0);
	}
}
