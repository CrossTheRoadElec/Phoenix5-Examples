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

namespace Hero_DisplayBoard
{
    //------------ The layout ------------//
    //int topBorder = _y;                           // top white border
    //int topStrt = _y + 1;                         // top line for top half
    //int top_End = _y + 1 + _top;                   //bottom line for top half
    //int btmStrt = _y + 1 + _top + 1;               // top line for bottom color
    //int btm_End = _y + 1 + _top + 1 + _btm;        // bottom line for bottom color
    //int btmBrdr = _y + 1 + _top + 1 + _btm + 1;    // bottom white line
    //int btmBrd2 = _y + 1 + _top + 1 + _btm + 2;    // second bottom line

    public class VerticalGauge
    {
        DisplayModule _displayModule;
        int _x, _y;

        int _height;
        int _width;

        int _top;
        int _btm;

        DisplayModule.RectSprite _backRect;
        DisplayModule.RectSprite _topRect;
        DisplayModule.RectSprite _btmRect;

        DisplayModule.Color _topCol;
        DisplayModule.Color _btmCol;

        public VerticalGauge(DisplayModule displayModule, int x, int y, int height, int width, DisplayModule.Color topCol, DisplayModule.Color btmCol)
        {
            _displayModule = displayModule;
            _x = x;
            _y = y;
            _height = height;
            _width = width;

            _top = 0;
            _btm = height;

            _topCol = topCol;
            _btmCol = btmCol;

            _backRect = _displayModule.AddRectSprite(DisplayModule.Color.White, x, _y, width + 2,  _top +  _btm + 5);

            _topRect = _displayModule.AddRectSprite(_topCol, _x + 1, _y + 1, _width, _top + 1);

            _btmRect = _displayModule.AddRectSprite(_btmCol, _x + 1, _y + 1 + _top + 1, _width, _btm + 1);
        }
        public int Value
        {
            set
            {
                _topRect.BeginUpdate();
                _btmRect.BeginUpdate();

                _top = value;
                if (_top > _height) _top = _height;
                if (_top < 0) _top = 0;
                _btm = _height - _top;

                _topRect.SetPosition(_x + 1, _y + 1);
                _topRect.SetSize(_width, _top + 1 );

                _btmRect.SetPosition(_x + 1, _y + 1 + _top + 1);
                _btmRect.SetSize(_width, _btm + 1 );

                _topRect.EndUpdate();
                _btmRect.EndUpdate();
            }
            get { return _top; }
        }
        public int MaxValue
        {
            get
            {
                return _height;
            }
        }
    }
}
