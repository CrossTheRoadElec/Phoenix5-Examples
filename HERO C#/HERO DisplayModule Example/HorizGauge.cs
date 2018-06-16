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

namespace DisplayModule_Example
{
    //------------ The layout ------------//
    //int LeftBorder = _x;                           // left white border
    //int LeftStrt = _x + 1;                         // left line for left half
    //int Left_End = _x + 1 + _left;                   //right line for left half
    //int RightStrt = _x + 1 + _left + 1;               // left line for right color
    //int Right_End = _x + 1 + _left + 1 + _rght;        // right line for right color
    //int RightBrdr = _x + 1 + _left + 1 + _rght + 1;    // right white line

    public class HorizGauge
    {
        DisplayModule _displayModule;
        int _x, _y;

        int _height;
        int _width;

        int _left;
        int _rght;

        DisplayModule.RectSprite _backRect;
        DisplayModule.RectSprite _leftRect;
        DisplayModule.RectSprite _rghtRect;

        DisplayModule.Color _leftCol;
        DisplayModule.Color _rghtCol;

        public HorizGauge(DisplayModule displayModule, int x, int y, int height, int width, DisplayModule.Color topCol, DisplayModule.Color btmCol)
        {
            _displayModule = displayModule;
            _x = x;
            _y = y;
            _height = height;
            _width = width;

            _left = 0;
            _rght = width;

            _leftCol = topCol;
            _rghtCol = btmCol;

            _backRect = _displayModule.AddRectSprite(DisplayModule.Color.White, x, _y, _left + _rght + 4, _height + 2);

            _leftRect = _displayModule.AddRectSprite(_leftCol, _x + 1, _y + 1, _left + 1, _height);

            _rghtRect = _displayModule.AddRectSprite(_rghtCol, _x + 1 + _left + 1, _y + 1, _rght + 1, _height);
        }
        public int Value
        {
            set
            {
                _leftRect.BeginUpdate();
                _rghtRect.BeginUpdate();

                _left = value;
                if (_left > _width) _left = _width;
                if (_left < 0) _left = 0;
                _rght = _width - _left;

                _leftRect.SetPosition(_x + 1, _y + 1);
                _leftRect.SetSize(_left + 1, _height);

                _rghtRect.SetPosition(_x + 1 + _left + 1, _y + 1);
                _rghtRect.SetSize(_rght + 1, _height);

                _leftRect.EndUpdate();
                _rghtRect.EndUpdate();
            }
            get { return _left; }
        }
        public int MaxValue
        {
            get
            {
                return _width;
            }
        }
    }
}
