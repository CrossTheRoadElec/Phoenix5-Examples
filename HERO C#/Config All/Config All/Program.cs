using Microsoft.SPOT;
using System.Threading;

using CTRE.Phoenix;
using CTRE.Phoenix.MotorControl.CAN;
using CTRE.Phoenix.Controller;
using CTRE.Phoenix.LowLevel;
using CTRE.Phoenix.Sensors;
using System;

/* This project may be used to test the GetAllConfigs, ConfigAllSettings, and ConfigFactoryDefault
 * Functions. Note that this project requires at least firmware 3.11 on Victors/Talons
 * for full function. Also, if firmware greater than 0.41 on the pigeon and 0.42 on the canfier
 * isn't used, the pigeon/canifier won't retain configs on reboot.
 * Some recommended tests:
 *   1. Set to custom configs and then read configs. Confirm that read and write are the same.
 *   2. Set to factory default configs and then read configs and confirm they are what is expected.
 *   3. Set to custom configs and then restart devices. Confirm that all configs persist between
 *   reboots. (See above note about pigeon and CANifier firmware)
 */

namespace Config_All
{
    public class Program
    {
        static RobotApplication _robotApp = new RobotApplication();

        public static void Main()
        {
            _robotApp.init();

            while (true)
            {
                _robotApp.run();

                Thread.Sleep(10);
            }
        }
    }

    public class RobotApplication
    {
        TalonSRX _talon = new TalonSRX(23);
        VictorSPX _victor = new VictorSPX(2);
        PigeonIMU _pigeon = new PigeonIMU(3);
        CANifier _canifier = new CANifier(4);

        /** Use a USB gamepad plugged into the HERO */
        GameController _gamepad = new GameController(UsbHostDevice.GetInstance());

        /** hold the current button values from gamepad*/
        bool[] _btns = new bool[10];

        configs _custom_configs = new configs();

        /** hold the last button values from gamepad, this makes detecting on-press events trivial */
        bool[] _btnsLast = new bool[10];

        public void init()
        {
        }

        public void run()
        {
            Loop10Ms();
        }

        void Loop10Ms()
        {
            /* get all the buttons */
            FillBtns(ref _btns);

            /* on button1 press read talon configs */
            if (_btns[1] && !_btnsLast[2])
            {
                Debug.Print("read talon");

                TalonSRXConfiguration read_talon;
                _talon.GetAllConfigs(out read_talon);

                Debug.Print(read_talon.ToString("_talon"));
            }
            /* on button2 press read victor configs */
            else if (_btns[2] && !_btnsLast[2])
            {
                Debug.Print("read victor");

                VictorSPXConfiguration read_victor;
                _victor.GetAllConfigs(out read_victor);

                Debug.Print(read_victor.ToString("_victor"));
            }
            /* on button3 press read pigeon configs */
            else if (_btns[3] && !_btnsLast[3])
            {

                Debug.Print("read pigeon");

                PigeonIMUConfiguration read_pigeon;
                _pigeon.GetAllConfigs(out read_pigeon);

                Debug.Print(read_pigeon.ToString("_pigeon"));

            }
            /* on button4 press read canifier configs */
            else if (_btns[4] && !_btnsLast[4])
            {
                Debug.Print("read canifier");

                CANifierConfiguration read_canifier;
                _canifier.GetAllConfigs(out read_canifier);

                Debug.Print(read_canifier.ToString("_canifier"));
            }
            /* on button5 press set custom configs */
            else if (_btns[5] && !_btnsLast[5])
            {
                Debug.Print("custom config start");

                _talon.ConfigAllSettings(_custom_configs._talon);
                _victor.ConfigAllSettings(_custom_configs._victor);
                _pigeon.ConfigAllSettings(_custom_configs._pigeon);
                _canifier.ConfigAllSettings(_custom_configs._canifier);

                Debug.Print("custom config finish");
            }
            /* on button6 press set factory default */
            else if (_btns[6] && !_btnsLast[6])
            {
                Debug.Print("factory default start");

				_talon.ConfigFactoryDefault();
                _victor.ConfigFactoryDefault();
                _pigeon.ConfigFactoryDefault();
                _canifier.ConfigFactoryDefault();

				Debug.Print("factory default finish");
            }
            /* set last presses */
            _btnsLast = (bool[])_btns.Clone();
        }

        /** throw all the gamepad buttons into an array */
        void FillBtns(ref bool[] btns)
        {
            for (uint i = 1; i < btns.Length; ++i)
            {
                btns[i] = _gamepad.GetButton(i);
            }
        }
    }
}
