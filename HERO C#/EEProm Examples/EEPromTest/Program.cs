
//#define ReadOnly
//Uncomment the line above to only read the data to prove it is persistant across boot cycles 
using System;
using System.Threading;
using Microsoft.SPOT;
using Microsoft.SPOT.Hardware;
using CTRE.HERO;
namespace EEProm
{
    public class Program
    {
        public static void Main()
        {
            OnboardEEPROM e = OnboardEEPROM.Instance;
            byte[] tofill = new byte[256];
            byte[] readData = new byte[256];

            for (int i = 0; i < tofill.Length; i++)
            {
                tofill[i] = (byte)33;// fill our byte array with values
            }
# if(!ReadOnly)
            for (int sector =0;  sector < 512; sector++)// go through the entire storage  512 * 4kb = the 16mbit storage that the hero has 
            {
                int current = sector + 1;
                Debug.Print("Erasing sector "+ current + "/512");

                e.Erase4KB(GetAddress(sector));//erase 
                e.Read(GetAddress(sector), readData);//readback erased bytearray to check for erase
                for(int x = 0; x < readData.Length; x++)
                {
                    if (readData[x] != 255)// check that storage has been erased  
                        Debug.Print("address"+ GetAddress(sector) + " not cleared");// this should never happen

                }

            }
            // all stored data has now been erased 

            //now lets fill it with new data
            for (int sector = 0; sector < 512; sector++)// go through the entire storage  512 * 4kb = the 16mbit storage that the hero has 
            {
                int current = sector + 1;
                Debug.Print("filling sector " + current + "/512");
              

                e.Write(GetAddress(sector),tofill);//fill
                e.Read(GetAddress(sector), readData);//readback filled bytearray to check for fill
                for (int x = 0; x < readData.Length; x++)
                {
                    if (readData[x] != 33)// check that storage has been filled  
                        Debug.Print("address" + GetAddress(sector) + "not filled correctly ");// this should never happen

                }

            }
            //memory has been filled
#endif


            // print all 16mbit of values
            for (int sector = 0; sector < 512; sector++)// go through the entire storage  512 * 4kb = the 16mbit storage that the hero has 
            {
                
                int current = sector + 1;

                e.Read(GetAddress(sector), readData);//read data 
                bool allGood = true;
                for (int x = 0; x < readData.Length; x++)
                {

                    if (readData[x] != 33)
                    {
                        allGood = false;
                        Debug.Print("printing sector " + current + "/512 " + " address " + GetAddress(sector) + " byte " + x + " value " + readData[x]);

                    }


                }
                if (allGood)
                {
                    Debug.Print("all data in sector " + current + "/512 is correct");
                }

            }// to see that memory is persistant comment out "define ReadOnly at the top and run code"
            while (true)

            {
                // do whatever here
                System.Threading.Thread.Sleep(100);
            }
        }
		static ulong GetAddress(int sector)// returns sector address invalid requests will recive an  address of 0
		{
			ulong retval = 0;
			if (sector == 0)
				retval = 1;
			else if (sector > 512)
				retval = 0;
			else
				retval = (ulong)(sector * 4096);
			return retval;
		}

	}
}
