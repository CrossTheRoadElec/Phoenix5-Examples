/*
*  Software License Agreement
*
* Copyright (C) Cross The Road Electronics.  All rights
* reserved.
* 
* Cross The Road Electronics (CTRE) licenses to you the right to 
* use, publish, and distribute copies of CRF (Cross The Road) firmware files (*.crf) and Software
* API Libraries ONLY when in use with Cross The Road Electronics hardware products.
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

namespace CTRE
{
    /// <summary>
    /// General listening serial class with a background thread and hook for deserializing frames.
    /// </summary>
    public class SerialClient
    {
        private Thread[] _thrd = new Thread[1];

        /** Serial object, this is constructed on the serial number. */
        System.IO.Ports.SerialPort _uart;

        /** Cache for reading out bytes in serial driver. */
        byte[] _rx = new byte[1024];

        ByteRingBuffer _rb = new ByteRingBuffer(1024);

        public delegate void ProcessData(ByteRingBuffer ringBuffer);

        ProcessData _processData;

        public SerialClient(string uartPort, ProcessData processData)
        {
            _processData = processData;

            /* open the UART, select the com port based on the desired gadgeteer port.*/
            _uart = new System.IO.Ports.SerialPort(uartPort, 115200);
            _uart.Open();

            _thrd[0] = new Thread(BackGround);
            _thrd[0].Start();
        }
        public void Write(byte [] buffer)
        {
            _uart.Write(buffer, 0, buffer.Length);
        }
        private void BackGround()
        {
            /* loop forever */
            while (true)
            {
                /* read bytes out of uart */
                if (_uart.BytesToRead > 0)
                {
                    int readCnt = _uart.Read(_rx, 0, (int)_rb.RemainingCapacity);
                    _rb.Push(_rx, readCnt);
                }

                /* pass to child object */
                _processData(_rb);

                /* wait a bit, keep the main loop time constant, this way you can add to this example (motor control for example). */
                System.Threading.Thread.Sleep(1);
            }
        }
        protected System.IO.Ports.SerialPort SerialPort
        {
            get
            {
                return _uart;
            }
        }
    }
}
