using System;
using Microsoft.SPOT;

namespace Hero_PixyDrive
{
    class ColorSequencer
    {
        /** Color codes for common colors */
        public const uint RED = 0xFF0000;
        public const uint Smoother1 = 0xFF4400;
        public const uint Transition1 = 0xFF8800;
        public const uint Smoother2 = 0xFFCC00;
        public const uint YELLOW = 0xFFFF00;
        public const uint Smoother3 = 0xCCFF00;
        public const uint Transition2 = 0x88FF00;
        public const uint Smoother4 = 0x44FF00;
        public const uint GREEN = 0x00FF00;
        public const uint Smoother5 = 0x00FF44;
        public const uint Transition3 = 0x00FF88;
        public const uint Smoother6 = 0x00FFCC;
        public const uint CYAN = 0x00FFFF;
        public const uint Smoother7 = 0x00CCFF;
        public const uint Transition4 = 0x0088FF;
        public const uint Smoother8 = 0x0044FF;
        public const uint BLUE = 0x0000FF;
        public const uint Smoother9 = 0x4400FF;
        public const uint Transition5 = 0x8800FF;
        public const uint Smoother10 = 0xCC00FF;
        public const uint MAGENTA = 0xFF00FF;
        public const uint Smoother11 = 0xFF00CC;
        public const uint Transition6 = 0xFF0088;
        public const uint Smoother12 = 0xFF0044;

        public const uint WHITE = 0xFFFFFF;
        public const uint PURPLE = 0x800080;
        public const uint ORANGE = 0xFF3a00;
        public const uint PINK = 0xFF6065;
        public const uint OFF = 0x000000;

        uint[] _colorSequence =
        {
            RED,
            Smoother1,
            Transition1,
            Smoother2,
            YELLOW,
            Smoother3,
            Transition2,
            Smoother4,
            GREEN,
            Smoother5,
            Transition3,
            Smoother6,
            CYAN,
            Smoother7,
            Transition4,
            Smoother8,
            BLUE,
            Smoother9,
            Transition5,
            Smoother10,
            MAGENTA,
            Smoother11,
            Transition6,
            Smoother12
        };
        /// <summary> Color index in color sequence.</summary>
        uint _i = 0;
        uint _j = 1;


        float[] _rgb = new float[3];

        public ColorSequencer()
        {
            Convert(_colorSequence[_i], _rgb);
        }

        void Convert(uint color, out float r, out float g, out float b)
        {
            uint temp = color;

            b = temp & 0xFF;
            g = (temp >> 8) & 0xFF;
            r = (temp >> 16) & 0xFF;

            r *= 1f / 255f;
            g *= 1f / 255f;
            b *= 1f / 255f;
        }

        void Convert(uint color, float[] arr)
        {
            uint temp = color;

            arr[2] = temp & 0xFF;
            arr[1] = (temp >> 8) & 0xFF;
            arr[0] = (temp >> 16) & 0xFF;

            arr[0] *= 1f / 255f;
            arr[1] *= 1f / 255f;
            arr[2] *= 1f / 255f;
        }

        bool IsEqual(float[] f1, float[] f2)
        {

            float[] delta = { 0, 0, 0 };
            delta[0] = f1[0] - f2[0];
            delta[1] = f1[1] - f2[1];
            delta[2] = f1[2] - f2[2];

            if (delta[0] < 0) { delta[0] *= -1f; }
            if (delta[1] < 0) { delta[1] *= -1f; }
            if (delta[2] < 0) { delta[2] *= -1f; }

            if (delta[0] > 0.01) return false;
            if (delta[1] > 0.01) return false;
            if (delta[2] > 0.01) return false;

            return true;
        }


        public void Process()
        {
            float[] p1 = { 0, 0, 0 };
            float[] p2 = { 0, 0, 0 };

            Convert(_colorSequence[_i], p1);
            Convert(_colorSequence[_j], p2);

            float[] delta = { 0, 0, 0 };

            delta[0] = p2[0] - p1[0];
            delta[1] = p2[1] - p1[1];
            delta[2] = p2[2] - p1[2];

            float scale = 1f / 16f;
            delta[0] *= scale;
            delta[1] *= scale;
            delta[2] *= scale;

            _rgb[0] += delta[0];
            _rgb[1] += delta[1];
            _rgb[2] += delta[2];

            if (IsEqual(_rgb, p2))
            {
                ++_i;
                ++_j;
                if (_i >= _colorSequence.Length)
                    _i = 0;
                if (_j >= _colorSequence.Length)
                    _j = 0;
            }
        }

        public float Red
        {
            get { return _rgb[0]; }
        }
        public float Green
        {
            get { return _rgb[1]; }
        }
        public float Blue
        {
            get { return _rgb[2]; }
        }

    }
}
