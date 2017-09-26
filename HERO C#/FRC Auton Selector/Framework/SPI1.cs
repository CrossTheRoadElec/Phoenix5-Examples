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
using Microsoft.SPOT.Hardware;

namespace Hardware
{
    public static class SPI1
    {
        private static Microsoft.SPOT.Hardware.SPI _spi = null;

        /* start with a dummy config */
        private static SPI.Configuration _config = new SPI.Configuration(Cpu.Pin.GPIO_NONE, false, 0, 0, false, true, 12000, SPI.SPI_module.SPI1);

        public static SPI Get(SPI.Configuration config)
        {
            Config = config;
            return _spi;
        }


        private static SPI Spi
        {
            get
            {
                if (_spi == null)
                    _spi = new Microsoft.SPOT.Hardware.SPI(_config);
                return _spi;
            }
        }
        private static SPI.Configuration Config
        {
            set
            {
                if (_config == value)
                {
                    /* nothing to do */
                }
                else
                {
                    Spi.Config = value;
                }
            }
        }
    }
}
