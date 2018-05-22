using System;
using System.Threading;
using Microsoft.SPOT.Hardware;

namespace CTRE
{
    namespace HERO
    {
        namespace Module
        {
            public class SonarModule
            {
                private static I2CDevice MyI2C = null;
                private static I2CDevice.I2CTransaction[] WriteCommand;
                private static I2CDevice.I2CTransaction[] ReadCommand;
                static int ReadCheck = 0;

                public enum RangeType
                {
                    Inches = 0x50,
                    Centimeters = 0x51,
                    MicroSeconds = 0x52
                }

                //I2C write function that takes the address and data
                private static void I2CWrite(byte Address, byte Data)
                {
                    WriteCommand = new I2CDevice.I2CTransaction[1];
                    WriteCommand[0] = I2CDevice.CreateWriteTransaction(new byte[2]);
                    WriteCommand[0].Buffer[0] = Address;
                    WriteCommand[0].Buffer[1] = Data;
                    MyI2C.Execute(WriteCommand, 100);
                }
                //I2C read function that takes the address, and buffer, and returns amount of transactions
                private static int I2CRead(byte Address, byte[] Data)
                {
                    ReadCommand = new I2CDevice.I2CTransaction[2];
                    ReadCommand[0] = I2CDevice.CreateWriteTransaction(new byte[] { Address });
                    ReadCommand[1] = I2CDevice.CreateReadTransaction(Data);
                    ReadCheck = MyI2C.Execute(ReadCommand, 100);
                    return ReadCheck;
                }

                //Constuctor that takes device address and clockrate
                public SonarModule(byte DeviceAddress, int ClockRate)
                {
                    I2CDevice.Configuration SonarConfig = new I2CDevice.Configuration(DeviceAddress, ClockRate);
                    MyI2C = new I2CDevice(SonarConfig);
                }

                //Set the analogue gain. If SRF10, Gain should be 0-5. If SRF08, Gain should be 0-20.
                public void SetGain(byte Gain)
                {
                    byte GainAddress = 0x01;
                    I2CWrite(GainAddress, Gain);
                }

                //Set the Range Distance once prior to ranging
                // 24 = 1m
                // 48 = 2m
                // 93 = 4m
                // 255 = 11m (Max Distance)
                // ((Distance x 43mm) + 43mm)
                public void SetDistance(byte Distance)
                {
                    byte RangeAddress = 0x02;
                    I2CWrite(RangeAddress, Distance);
                }

                //Call this to range once
                public void InitRanging(RangeType RangeType)
                {
                    byte CommandAddress = 0x00;
                    byte type = (byte)RangeType;
                    I2CWrite(CommandAddress, type);
                }

                //Call This to read the range once, Still need a timeout
                public uint ReadRange()
                {
                    //int Count = 0;
                    uint SonarSample = 0;
                    byte HighAddress = 0x02;
                    byte LowAddress = 0x03;
                    byte[] HighByte = new byte[1];
                    byte[] LowByte = new byte[1];

                    Thread.Sleep(30);

                    I2CRead(HighAddress, HighByte);
                    I2CRead(LowAddress, LowByte);

                    SonarSample = HighByte[0];
                    SonarSample <<= 8;
                    SonarSample |= LowByte[0];

                    return SonarSample;
                }
            }
        }
    }
}