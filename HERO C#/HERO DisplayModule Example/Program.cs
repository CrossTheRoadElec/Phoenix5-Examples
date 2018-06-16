/**
 * Example HERO application with Display Module on port8, and USB gamepad inserted into HERO.
 * Bar graphs and text will update when user changes button states and axis values on gamepad.
 * Tested with Phoenix Framework 5.6.0.0 (Phoenix NETMF: 5.1.5.0)
 */
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
using Microsoft.SPOT;
using CTRE.Gadgeteer.Module;
using CTRE.Phoenix.Controller;
using CTRE.Phoenix;

namespace DisplayModule_Example
{
    public class Program
    {
        GameController _gamepad = new GameController(UsbHostDevice.GetInstance());

        DisplayModule _displayModule = new DisplayModule(CTRE.HERO.IO.Port1, DisplayModule.OrientationType.Landscape);

        /* lets pick a font */
        Font _smallFont = Properties.Resources.GetFont(Properties.Resources.FontResources.small);
        Font _bigFont = Properties.Resources.GetFont(Properties.Resources.FontResources.NinaB);

        VerticalGauge _leftY, _rightY;
        HorizGauge _leftX, _rightX;
        DisplayModule.ResourceImageSprite _leftCrossHair, _rightCrossHair;
        DisplayModule.LabelSprite _labelTitle, _labelBtn;


        public void UpdateGauge(HorizGauge gauge, float axis)
        {
            axis += 1.0f; // [0,2]
            axis *= 0.5f; // [0,1]
            gauge.Value = (int)(axis * gauge.MaxValue);
        }
        public void UpdateGauge(VerticalGauge gauge, float axis)
        {
            axis += 1.0f; // [0,2]
            axis *= 0.5f; // [0,1]
            gauge.Value = (int)(axis * gauge.MaxValue);
        }
        public int GetFirstButton(GameController gamepad)
        {
            for (uint i = 1; i < 16 ;++i)
            {
                if (gamepad.GetButton(i))
                    return (int)i;
            }
            return -1;
        }
        public void RunForever()
        {

            _leftY = new VerticalGauge(_displayModule, 5, 5, 30, 10, DisplayModule.Color.Cyan, DisplayModule.Color.Blue);
            _rightY = new VerticalGauge(_displayModule, 135, 5, 30, 10, DisplayModule.Color.Yellow, DisplayModule.Color.Red);


            _leftX = new HorizGauge(_displayModule, 35, 30, 10, 30, DisplayModule.Color.Green, DisplayModule.Color.Magenta);
            _rightX = new HorizGauge(_displayModule, 85, 30, 10, 30, DisplayModule.Color.Blue, DisplayModule.Color.Orange);

            _leftCrossHair = _displayModule.AddResourceImageSprite(
                                                           DisplayModule_Example.Properties.Resources.ResourceManager,
                                                           DisplayModule_Example.Properties.Resources.BinaryResources.ch2,
                                                           Bitmap.BitmapImageType.Jpeg,
                                                           30, 100);

            _rightCrossHair = _displayModule.AddResourceImageSprite(
                                                           DisplayModule_Example.Properties.Resources.ResourceManager,
                                                           DisplayModule_Example.Properties.Resources.BinaryResources.ch2,
                                                           Bitmap.BitmapImageType.Jpeg,
                                                           100, 100);

            _labelTitle = _displayModule.AddLabelSprite(_bigFont, DisplayModule.Color.White, 40, 0, 80, 16);

            _labelBtn = _displayModule.AddLabelSprite(_smallFont, DisplayModule.Color.White, 30, 50, 100, 15);

			while (true)
			{
                UpdateGauge(_leftX, _gamepad.GetAxis(0));
                UpdateGauge(_leftY, _gamepad.GetAxis(1));
                UpdateGauge(_rightX, _gamepad.GetAxis(2));
                UpdateGauge(_rightY, _gamepad.GetAxis(5));

                _leftCrossHair.SetPosition((int)(30 + 15 * _gamepad.GetAxis(0)), 100 + (int)(15 * _gamepad.GetAxis(1)));
                _rightCrossHair.SetPosition((int)(100 + 15 * _gamepad.GetAxis(2)), 100 + (int)(15 * _gamepad.GetAxis(5)));

                if (_gamepad.GetConnectionStatus() == UsbDeviceConnection.Connected)
                {
                    _labelTitle.SetText("Connected");
                    _labelTitle.SetColor(DisplayModule.Color.Green);
                }
                else
                {
                    _labelTitle.SetText("No Gamepad");
                    _labelTitle.SetColor(DisplayModule.Color.Red);
                }

                int idx = GetFirstButton(_gamepad);
                if (idx < 0)
                {
                    _labelBtn.SetColor((DisplayModule.Color)0xA0A0A0); // gray RGB
                    _labelBtn.SetText("No Buttons");
                }
                else
                {
                    switch (idx % 4)
                    {
                        case 0: _labelBtn.SetColor(DisplayModule.Color.Cyan); break;
                        case 1: _labelBtn.SetColor(DisplayModule.Color.Green); break;
                        case 2: _labelBtn.SetColor(DisplayModule.Color.Red); break;
                        case 3: _labelBtn.SetColor(DisplayModule.Color.Yellow); break;

			}
                    _labelBtn.SetText("Pressed Button " + idx);
        }

                Thread.Sleep(10);
            }
        }
        public static void Main()
        {
            new Program().RunForever();
        }
    }
}
