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

using System;
using Microsoft.SPOT;
using Microsoft.SPOT.Hardware;


namespace Hero_Autonomous_Selector_Example
{
    public class OnboardEEPROM {
        //private SPI _SPIDevice;
        private const Cpu.Pin _ChipSelFlash = (Cpu.Pin)0x25;
        private SPI.Configuration _Configuration = new SPI.Configuration(_ChipSelFlash, false, 0, 0, false, true, 8000, SPI.SPI_module.SPI1);
        private static OnboardEEPROM _instance = null;

        public static OnboardEEPROM Instance {
            get {
                if (_instance == null) {
                    _instance = new OnboardEEPROM();
                }
                return _instance;
            }
        }

        //Configure SPI for the Onboard EEPROM
        private OnboardEEPROM()
        {
            _Configuration = new SPI.Configuration(_ChipSelFlash, false, 0, 0, false, true, 8000, SPI.SPI_module.SPI1);
            //_SPIDevice = new SPI(_Configuration);
        }

        private enum OpCode //Instuctions from SST25VF016B-75-4I-QAF
        {
            Read = 0x03,
            HighSpeedRead = 0x0b,
            _4KBErase = 0x20,
            _32KBErase = 0x52,
            _64KBErase = 0xd8,
            _ChipErase = 0x60,
            ByteProgram = 0x02,
            AAIWordProgram = 0xad,
            RDSR_ReadStatusReg = 0x05,
            EWSR_EnableWriteStatReg = 0x50,
            WRSR_WriteStatusReg = 0x01,
            WREN_WriteEn = 0x06,
            WRDI_WriteDisable = 0x04,
            RDID_ReadId = 0xab,
            JEDECID = 0x9f,
        };
        //Read the status register and returns the byte
        private byte ReadStatusRegister()
        {
            byte[] Temp = new byte[1];
            byte[] TempIn = new byte[1];
            Temp[0] = (byte)OpCode.RDSR_ReadStatusReg;

            Microsoft.SPOT.Hardware.SPI _SPIDevice = Hardware.SPI1.Get(_Configuration);

            _SPIDevice.WriteRead(Temp, 0, 1, TempIn, 0, 1, 1);
            return TempIn[0];
        }
        //Good

        //Enables Write Status Register
        private void EWSR()
        {
            byte[] Temp = new byte[1];
            Temp[0] = (byte)OpCode.EWSR_EnableWriteStatReg;

            Microsoft.SPOT.Hardware.SPI _SPIDevice = Hardware.SPI1.Get(_Configuration);

            _SPIDevice.Write(Temp);
            Delay();
        }
        //Good

        //Writes a byte to the status register
        private void WRSR(byte StatusIn)
        {
            byte[] Temp = new byte[2];
            Temp[0] = (byte)OpCode.WRSR_WriteStatusReg;
            Temp[1] = StatusIn;

            Microsoft.SPOT.Hardware.SPI _SPIDevice = Hardware.SPI1.Get(_Configuration);

            _SPIDevice.Write(Temp);
            Delay();
        }
        //Good

        //Enables the Write Enable latch. Can also be used in place of EWSR
        private void WREN()
        {
            byte[] Temp = new byte[1];
            Temp[0] = (byte)OpCode.WREN_WriteEn;

            Microsoft.SPOT.Hardware.SPI _SPIDevice = Hardware.SPI1.Get(_Configuration);

            _SPIDevice.Write(Temp);
            Delay();
        }
        //Good

        //Disables the Write Enable latch
        private void WRDI()
        {
            byte[] Temp = new byte[1];
            Temp[0] = (byte)OpCode.WRDI_WriteDisable;

            Microsoft.SPOT.Hardware.SPI _SPIDevice = Hardware.SPI1.Get(_Configuration);

            _SPIDevice.Write(Temp);
            Delay();
        }
        //Good

        public uint ReadJedecID()
        {
            byte[] outBytes = new byte[4] { (byte)OpCode.JEDECID, 0x00, 0x00, 0x00 };
            byte[] inBytes = new byte[4];

            Microsoft.SPOT.Hardware.SPI _SPIDevice = Hardware.SPI1.Get(_Configuration);

            _SPIDevice.WriteRead(outBytes, inBytes);

            uint retval = inBytes[1];
            retval <<= 8;
            retval |= inBytes[2];
            retval <<= 8;
            retval |= inBytes[3];
            return retval;
        }


        //Read multiple address and store data into byte buffer;
        public void ReadMultiple(ulong Address, int Numb_of_Bytes, byte[] DestBuffer)
        {
            byte[] Temp = new byte[4];
            Wait_Busy();
            byte Byte1 = (byte)((Address & 0xFFFFFF) >> 16);
            byte Byte2 = (byte)((Address & 0xFFFF) >> 8);
            byte Byte3 = (byte)(Address & 0xFF);
            Temp[0] = (byte)OpCode.Read;
            Temp[1] = Byte1;
            Temp[2] = Byte2;
            Temp[3] = Byte3;

            Microsoft.SPOT.Hardware.SPI _SPIDevice = Hardware.SPI1.Get(_Configuration);

            _SPIDevice.WriteRead(Temp, 0, 4, DestBuffer, 0, Numb_of_Bytes, 4);
        }
        //Good


        //=======================================================================//

        private void Auto_Add_IncA(ulong Address, byte Data1, byte Data2)
        {
            byte[] Temp = new byte[6];
            byte Byte1 = (byte)((Address & 0xFFFFFF) >> 16);
            byte Byte2 = (byte)((Address & 0xFFFF) >> 8);
            byte Byte3 = (byte)(Address & 0xFF);
            Wait_Busy();
            WREN();
            Temp[0] = (byte)OpCode.AAIWordProgram;
            Temp[1] = Byte1;
            Temp[2] = Byte2;
            Temp[3] = Byte3;
            Temp[4] = Data1;
            Temp[5] = Data2;

            Microsoft.SPOT.Hardware.SPI _SPIDevice = Hardware.SPI1.Get(_Configuration);

            _SPIDevice.Write(Temp);
            Delay();
        }

        private void Auto_Add_IncB(byte Data1, byte Data2)
        {
            WREN_AAI_Check();
            byte[] Temp = new byte[3];
            Temp[0] = (byte)OpCode.AAIWordProgram;
            Temp[1] = Data1;
            Temp[2] = Data2;

            Microsoft.SPOT.Hardware.SPI _SPIDevice = Hardware.SPI1.Get(_Configuration);

            _SPIDevice.Write(Temp);
            Delay();
        }
        //Must be first write and writes a single 4 bytes or int
        public void WriteHisto(ulong Address, uint FirstData, uint[] DataToWrite, int length, uint Checksum)
        {
            byte[] Datas = new byte[4];
            //Int Dat Conveted
            Datas[0] = (byte)((FirstData >> 24) & 0xFF);
            Datas[1] = (byte)((FirstData >> 16) & 0xFF);
            Datas[2] = (byte)((FirstData >> 8) & 0xFF);
            Datas[3] = (byte)(FirstData & 0xFF);

            //Clear the Block protection bits to allow writes
            WREN();
            Delay();
            WRSR(0x00);
            Delay();
            //WREN again and clear the sector...
            Sector_Erase(Address);//Sector must be deleted prior to writing to it
            Auto_Add_IncA(Address, Datas[0], Datas[1]);
            Auto_Add_IncB(Datas[2], Datas[3]);

            //public void WriteHistoData(uint[] DataToWrite, int length)        
            for (int i = 0; i < length; i++) {
                uint CurrentData = DataToWrite[i];
                byte[] temp = new byte[4];
                temp[0] = (byte)((CurrentData >> 24) & 0xFF);
                temp[1] = (byte)((CurrentData >> 16) & 0xFF);
                temp[2] = (byte)((CurrentData >> 8) & 0xFF);
                temp[3] = (byte)(CurrentData & 0xFF);
                Auto_Add_IncB(temp[0], temp[1]);
                Auto_Add_IncB(temp[2], temp[3]);
            }

            //public void WriteHistoFinish(uint Checksum)          
            byte[] Data4 = new byte[4];
            Data4[0] = (byte)((Checksum >> 24) & 0xFF);
            Data4[1] = (byte)((Checksum >> 16) & 0xFF);
            Data4[2] = (byte)((Checksum >> 8) & 0xFF);
            Data4[3] = (byte)(Checksum & 0xFF);
            Auto_Add_IncB(Data4[0], Data4[1]);
            Auto_Add_IncB(Data4[2], Data4[3]);
            //This is the reason why this function is differnt
            WRDI();
        }

        //Sector Erases the chip
        private void Sector_Erase(ulong Address)
        {
            byte[] Temp = new byte[4];
            Wait_Busy();
            WREN();
            Temp[0] = (byte)OpCode._4KBErase;
            Temp[1] = (byte)((Address & 0xFFFFFF) >> 16);
            Temp[2] = (byte)((Address & 0xFFFF) >> 8);
            Temp[3] = (byte)(Address & 0xFF);

            Microsoft.SPOT.Hardware.SPI _SPIDevice = Hardware.SPI1.Get(_Configuration);

            _SPIDevice.Write(Temp);
            DelayErase();
        }

        //Checks the status register before proceeding
        private void Wait_Busy(int timeout = 100)
        {
            while ((ReadStatusRegister() & 0x03) == 0x03) //Waste time until not busy
            {
                System.Threading.Thread.Sleep(10);

                timeout -= 10;
                if (timeout < 0) {
                    break;
                }
            }
        }

        private void WREN_AAI_Check()
        {
            byte Byte;
            Byte = ReadStatusRegister();  /* read the status register */
            if (Byte != 0x42)       /* verify that AAI and WEL bit is set */
            {
                //while (true) // todo remove this, only print once.
                {
                    Debug.Print("Restart? Need to change check");
                }
            }
        }

        void DelayErase()
        {
            System.Threading.Thread.Sleep(100);
        }
        void Delay()
        {
            System.Threading.Thread.Sleep(1);
        }

        /**
         * General read function
         * @author Ozrien
         */
        public void ReadBytes(ulong Address, byte[] data, int length)
        {
            /* use Gia's routine, not sure what the smallest supported unit is */
            ReadMultiple(Address, length, data);
        }
        /**
         * General write function
         * length must be a multiple of four for some reason.  Not sure if this is intentional or just an artifact
         * of how Gia wrote it. 
         * @author Ozrien
         */
        public void WriteBytes(ulong Address, byte[] data, int length)
        {
            byte[] temp = new byte[4];

            int i = 0;
            temp[0] = data[i++];
            temp[1] = data[i++];
            temp[2] = data[i++];
            temp[3] = data[i++];

            //Clear the Block protection bits to allow writes
            WREN();
            Delay();
            WRSR(0x00);
            Delay();
            //WREN again and clear the sector...
            Sector_Erase(Address);//Sector must be deleted prior to writing to it
            Auto_Add_IncA(Address, temp[0], temp[1]);
            Auto_Add_IncB(temp[2], temp[3]);

            //public void WriteHistoData(uint[] DataToWrite, int length)        
            while (i < length) {
                temp[0] = data[i++];
                temp[1] = data[i++];
                temp[2] = data[i++];
                temp[3] = data[i++];
                Auto_Add_IncB(temp[0], temp[1]);
                Auto_Add_IncB(temp[2], temp[3]);
            }

            ////public void WriteHistoFinish(uint Checksum)          
            //byte[] Data4 = new byte[4];
            //Data4[0] = data[i++];
            //Data4[1] = data[i++];
            //Data4[2] = data[i++];
            //Data4[3] = data[i++];
            //Auto_Add_IncB(Data4[0], Data4[1]);
            //Auto_Add_IncB(Data4[2], Data4[3]);


            //This is the reason why this function is differnt
            WRDI();
        }
    }
}