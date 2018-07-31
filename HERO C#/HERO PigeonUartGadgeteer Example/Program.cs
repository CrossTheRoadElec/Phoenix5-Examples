/**
 * Example HERO application can reads a serial port and echos the bytes back.
 * After deploying this application, the user can open a serial terminal and type while the HERO echoes the typed keys back.
 * Use a USB to UART (TTL) cable like the Adafruit Raspberry PI or FTDI-TTL cable.
 * Use device manager to figure out which serial port number to select in your PC terminal program.
 * HERO Gadgeteer Port 1 is used in this example, but can be changed at the top of Main().
 */
using Microsoft.SPOT;

namespace HERO_PigeonUartGadgeteer_Example
{
    public class Program
    {
        /** entry point of the application */
        public static void Main()
        {
            CTRE.PigeonUartGadgeteer _pigeonUartGadg = new CTRE.PigeonUartGadgeteer(CTRE.HERO.IO.Port4);

            while (true)
            {

                /* wait a bit, keep the main loop time constant, this way you can add to this example (motor control for example). */
                System.Threading.Thread.Sleep(10);

                Debug.Print("Yaw:" + _pigeonUartGadg.Yaw + "   " + "Pitch:" + _pigeonUartGadg.Pitch + "   " + "Roll:" + _pigeonUartGadg.Roll);
            }
        }
    }
}
