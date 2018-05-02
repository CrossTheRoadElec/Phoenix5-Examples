/**
 * Basic Example demonstrating the Gadgeteer Driver Module.
 * Tested with Logitech F310 USB Gamepad inserted into the HERO.
 * 
 * Use the mini-USB cable to deploy/debug.
 * 
 * The driver module is a low-side driver, which means the outputs
 * should be wired to the low side (or 'ground') of the device being
 * operated.  Driving the module output to ground creates a voltage difference
 * accross the device input power terminals, enabling the connected device.
 * Pulling the module output up to the high voltage level eliminates 
 * the voltage difference, disabling the connected device.
 * 
 * Use the 'X' button on the joystick to enable outputs 1 and 2 on the Driver Module.
 * 
 * IMPORTANT: This example requires the version of the SDK from the
 * Installer version 5.4.3.0 or higher.  There were several changes
 * and additions to multiple files in the SDK to enable generic module
 * classes, and these are required in addition to the basic
 * DriverModule.cs in the SDK.
 * 
 */

using System;
using Microsoft.SPOT;
using CTRE.Phoenix;
using CTRE.Phoenix.Controller;
using CTRE.Gadgeteer.Module;

namespace HERO_Driver_Module_Example
{
    public class Program
    {
        public static void Main()
        {
            //Gamepad for input
            GameController _gamepad = new GameController(UsbHostDevice.GetInstance());

            //Create the DriverModule object by giving it the port you plugged it in to.
            DriverModule driver = new DriverModule(CTRE.HERO.IO.Port5);

            //these just act as shorter names for the driveLow and pullUp states
            bool driveLow = DriverModule.OutputState.driveLow;
            bool pullUp = DriverModule.OutputState.pullUp;

            while (true)
            {  
                //When the 'X' button is pressed, enable outputs
                if (_gamepad.GetButton(1) == true)
                {
                    driver.Set(1, driveLow);
                    driver.Set(2, driveLow);
                }
                else
                {
                    driver.Set(1, pullUp);
                    driver.Set(2, pullUp);
                }

                System.Threading.Thread.Sleep(10);
            }
        }
    }
}
