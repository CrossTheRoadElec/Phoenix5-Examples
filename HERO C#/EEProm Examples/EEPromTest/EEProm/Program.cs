
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
            for (int max =1;  max < 512; max++)// go through the entire storage  512 * 4kb = the 16mbit storage that the hero has 
            {
                ulong ad = (ulong)(max * 4096);
                int current = max + 1;
                Debug.Print("Erasing sector "+ current + "/512");

                e.Erase4KB(ad);//erase 
                e.Read(ad, readData);//readback erased bytearray to check for erase
                for(int x = 0; x < readData.Length; x++)
                {
                    if (readData[x] != 255)// check that storage has been erased  
                        Debug.Print("address"+ad+"not cleared");// this should never happen

                }

            }
            // all stored data has now been erased 

            //now lets fill it with new data
            for (int max = 1; max < 512; max++)// go through the entire storage  512 * 4kb = the 16mbit storage that the hero has 
            {
                int current = max + 1;
                Debug.Print("filling sector " + current + "/512");
                ulong ad = (ulong)(max * 4096);

                e.Write(ad,tofill);//fill
                e.Read(ad, readData);//readback filled bytearray to check for fill
                for (int x = 0; x < readData.Length; x++)
                {
                    if (readData[x] != 33)// check that storage has been filled  
                        Debug.Print("address" + ad + "not filled correctly ");// this should never happen

                }

            }
            //memory has been filled
#endif


            // print all 16mbit of values
            for (int max = 1; max < 512; max++)// go through the entire storage  512 * 4kb = the 16mbit storage that the hero has 
            {
                ulong ad = (ulong)(max * 4096);
                int current = max + 1;

                e.Read(ad, readData);//read data 
                bool allGood = true;
                for (int x = 0; x < readData.Length; x++)
                {

                    if (readData[x] != 33)
                    {
                        allGood = false;
                        Debug.Print("printing sector " + current + "/512 " + " address " + ad + " byte " + x + " value " + readData[x]);

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
    }
}
