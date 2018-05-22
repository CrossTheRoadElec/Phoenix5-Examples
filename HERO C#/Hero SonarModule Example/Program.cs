//#define SETID  // Uncomment this line to set Addresses of Sonar Modules

using System;
using System.Threading;
using Microsoft.SPOT;
using Microsoft.SPOT.Hardware;
using CTRE.HERO.Module;

namespace Hero_SonarModule
{
    public class Program
    {
#if !SETID
        static SonarModule MySonar = new SonarModule(0x7F, 100);
#endif

        public static void Main()
        {
#if SETID
            while (true)
            {
                SonarModuleAddressChange what = new SonarModuleAddressChange(0x00, 0xFE); //Front is 0xFE, Side is 0xE0 (This is the default for SRF08)
                Thread.Sleep(10);
            }
#else
            //Best settings found so far:
            //SRF10 -> Gains: 6, Range: 48
            //SRF08 -> Gains: 0, Range: 93
            MySonar.SetGain(6);         //SRF08 0-16, SRF10 0- 31
            MySonar.SetDistance(48);   //SRFXX 0 - 255
            uint SonarRead = 0;
            while (true)
            {
                MySonar.InitRanging(SonarModule.RangeType.Centimeters);
                SonarRead = MySonar.ReadRange();
                Debug.Print("Read: " + SonarRead);
            }
#endif
        }
    }
}