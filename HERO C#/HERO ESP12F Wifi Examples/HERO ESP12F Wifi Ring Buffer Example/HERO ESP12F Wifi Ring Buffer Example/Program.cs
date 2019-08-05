/**
 * Example HERO application that configures the ESP12F Wifi Gadgeteer Board for Communication.
 * This application will take data received and echo it back to a set destination.
 * The ESP12F Communicates via UART and so bandwidth will be limited by the UART Baudrate.
 * The default baudrate of 115200 has been tested and is supported.  Additional baud rates are still being tested and should not currently be used.
 * HERO Gadgeteer Port 1 is used in this example, but can be changed at the top of Main().
 * Communication type and Access Point Settings can be changed at the top of Main().
 */


#define UDP
//#define TCP
//#define TCPServer


using System;
using System.Threading;
using Microsoft.SPOT;
using Microsoft.SPOT.Hardware;
using CTRE.Gadgeteer.Module;
using CTRE.HERO;

namespace HERO_WiFi_Ring_Buffer_Example
{
	public class Program
	{
		/** WiFi object, this is constructed on the HERO Gadgeteer Port. */
		static WiFiESP12F wifi;
		/** Ring buffer holding the bytes to transmit. */
		static byte[] _tx = new byte[1024];
		static int _txIn = 0;
		static int _txOut = 0;
		static int _txCnt = 0;
		/** Cache for reading out bytes in serial driver. */
		static byte[] _rx = new byte[1024];
		static byte[] _cache = new byte[1024];
		static int cacheSize;
		/** @return the maximum number of bytes we can read*/
		private static int CalcRemainingCap()
		{
			/* firs calc the remaining capacity in the ring buffer */
			int rem = _tx.Length - _txCnt;
			/* cap the return to the maximum capacity of the rx array */
			if (rem > _rx.Length)
				rem = _rx.Length;
			return rem;
		}
		/** @param received byte to push into ring buffer */
		private static void PushByte(byte datum)
		{
			_tx[_txIn] = datum;
			if (++_txIn >= _tx.Length)
				_txIn = 0;
			++_txCnt;
		}
		/** 
         * Pop the oldest byte out of the ring buffer.
         * Caller must ensure there is at least one byte to pop out by checking _txCnt.
         * @return the oldest byte in buffer.
         */
		private static byte PopByte()
		{
			byte retval = _tx[_txOut];
			if (++_txOut >= _tx.Length)
				_txOut = 0;
			--_txCnt;
			return retval;
		}
		private static void PopTX(byte[] array)
		{
			int temp = _txCnt;
			for (int i = 0; i < temp; i++)
			{
				array[i] = PopByte();
			}
		}

		/** entry point of the application */
		public static void Main()
		{

			wifi = new WiFiESP12F(IO.Port1);
			wifi.reset();
			Thread.Sleep(1000);
			wifi.setWifiMode(WiFiESP12F.wifiMode.SOFTAP);
			wifi.setAP("WifiTest", "Password1", 1, WiFiESP12F.SecurityType.WPA_WPA2_PSK);
			Thread.Sleep(1000);// Required after calling setAP




#if (UDP)
			wifi.startUDP(4, "192.168.4.2", 11000, 11001);

#endif
#if (TCPServer)
            wifi.openTCPServer(11001); 
#endif
#if (TCP)
            wifi.startTCP(0, "192.168.4.2", 11000); 
#endif

			/* temporary array */
			byte[] scratch = new byte[1];


			/* loop forever */

			while (true)
			{
				/* read bytes out of uart */
				if (wifi.processInput())
				{
					/*transfer processed data to cache*/
					cacheSize = wifi.transferDataCache(_cache);
					for (int j = 0; j < cacheSize; j++)
					{
						PushByte(_cache[j]);
					}
				}

				/* if there are bufferd bytes echo them back out */
				if (_txCnt > 0)
				{
					scratch = new byte[_txCnt];
					PopTX(scratch);


#if (UDP)
					wifi.sendUDP(4, scratch);
#endif
#if (TCPServer || TCP)
                    wifi.sendTCP(0, scratch); //The '0' id assumes you only have one connection on the TCP server. A list of connections can be obtained using 'AT+CIPSTATUS'.
#endif
				}
			}
		}
		/**
         * Helper routine for creating byte arrays from strings.
         * @param msg string message to covnert.
         * @return byte array version of string.
         */
		private static byte[] MakeByteArrayFromString(String msg)
		{
			byte[] retval = new byte[msg.Length];
			for (int i = 0; i < msg.Length; ++i)
				retval[i] = (byte)msg[i];
			return retval;
		}
	}
}