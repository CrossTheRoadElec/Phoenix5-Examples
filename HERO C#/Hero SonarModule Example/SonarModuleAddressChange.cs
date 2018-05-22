using System;
using System.Threading;
using Microsoft.SPOT;
using Microsoft.SPOT.Hardware;

namespace Hero_SonarModule
{
    class SonarModuleAddressChange
    {
        int AnyValue;
        I2CDevice MyI2C;
        //Was unable to make this stupd proof due to execpetions.
        //Input Current Addres in 7bit form and Desired Address in 8 bit form.
        public SonarModuleAddressChange(byte CurrentAddress, byte DesiredAddress)
        {
            I2CDevice.Configuration SonarConfig = new I2CDevice.Configuration(CurrentAddress, 100);
            if (MyI2C == null) { MyI2C = new I2CDevice(SonarConfig); }

            //Do a single write read to check if the address works
            byte[] Data = new byte[1];
            I2CDevice.I2CTransaction[] ReadCommand = new I2CDevice.I2CTransaction[2];
            ReadCommand[0] = I2CDevice.CreateWriteTransaction(new byte[] { 0x00 });
            ReadCommand[1] = I2CDevice.CreateReadTransaction(Data);
            AnyValue = MyI2C.Execute(ReadCommand, 100);

            if (AnyValue != 0) //11 for the SRF08 and 5 for the SRF10;
            {
                //Address was found and now we send change address commands
                I2CDevice.I2CTransaction[] WriteCommand = new I2CDevice.I2CTransaction[1];
                WriteCommand[0] = I2CDevice.CreateWriteTransaction(new byte[2]);
                WriteCommand[0].Buffer[0] = 0x00;
                WriteCommand[0].Buffer[1] = 0xA0;
                MyI2C.Execute(WriteCommand, 100);

                WriteCommand[0].Buffer[0] = 0x00;
                WriteCommand[0].Buffer[1] = 0xAA;
                MyI2C.Execute(WriteCommand, 100);

                WriteCommand[0].Buffer[0] = 0x00;
                WriteCommand[0].Buffer[1] = 0xA5;
                MyI2C.Execute(WriteCommand, 100);

                WriteCommand[0].Buffer[0] = 0x00;
                WriteCommand[0].Buffer[1] = DesiredAddress;
                MyI2C.Execute(WriteCommand, 100);

                Debug.Print("Success");
                Thread.Sleep(5000);
            }
            else
            {
                Debug.Print("Failed");
                Thread.Sleep(5000);
            }

            MyI2C.Dispose();
            MyI2C = null;
        }
    }
}
