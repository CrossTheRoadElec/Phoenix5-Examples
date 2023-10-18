using System.Threading;
using CTRE.Phoenix.LED;

namespace CANdleExample
{
    public class Program
    {
        public static void Main()
        {
			CANdle candle = new CANdle(0);

			candle.ConfigBrightnessScalar(0.1f); // Make LEDs 10% brightness

			/* Start with setLED in a rainbow for a couple second */
			candle.SetLEDs(0, 0, 0, startIdx: 0, count: 1);
			candle.SetLEDs(0, 0, 255, startIdx: 1, count: 1);
			candle.SetLEDs(0, 255, 0, startIdx: 2, count: 1);
			candle.SetLEDs(0, 255, 255, startIdx: 3, count: 1);
			candle.SetLEDs(255, 0, 0, startIdx: 4, count: 1);
			candle.SetLEDs(255, 0, 255, startIdx: 5, count: 1);
			candle.SetLEDs(255, 255, 0, startIdx: 6, count: 1);
			candle.SetLEDs(255, 255, 255, startIdx: 7, count: 1);
			Thread.Sleep(5000);


			/* Loop through some animations every 1 second for the 8 on-board LEDs */
			int animationIndex = 0;
            /* loop forever */
            while (true)
            {
				Animation toAnimate;
				switch(animationIndex)
				{
					/* If animationIndex is not valid, make it 0 and start from the beginning */
					default:
						animationIndex = 0;
						goto case 0;

					/* Normal cases from here */
					case 0:
						toAnimate = new RainbowAnimation();
						animationIndex++;
						break;
					case 1:
						toAnimate = new LarsonAnimation(128, 128, 0, numLed: 8);
						animationIndex++;
						break;
					case 2:
						toAnimate = new FireAnimation(numLed: 8);
						animationIndex++;
						break;
					case 3:
						toAnimate = new TwinkleAnimation(0, 255, 128, numLed: 8);
						animationIndex = 0; // Start over from the top
						break;
				}

				candle.Animate(toAnimate);

				/* wait a bit */
				Thread.Sleep(1000);
            }
        }
    }
}
