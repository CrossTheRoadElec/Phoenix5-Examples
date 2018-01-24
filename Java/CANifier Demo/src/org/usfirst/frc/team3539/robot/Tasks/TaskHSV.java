package org.usfirst.frc.team3539.robot.Tasks;

import com.ctre.phoenix.ILoopable;
import com.ctre.phoenix.CANifier;
import org.usfirst.frc.team3539.robot.Framework.*;
import org.usfirst.frc.team3539.robot.Platform.*;

public class TaskHSV implements ILoopable {
	public float Hue;
	public float Saturation;
	public float Value;

	private static float _rgb[] = new float[3];

	private MovingAverage _averageR = new MovingAverage(10);
	private MovingAverage _averageG = new MovingAverage(10);
	private MovingAverage _averageB = new MovingAverage(10);

	/* ILoopable */
	public void onStart() {

	}

	public void onStop() {

	}

	public boolean isDone() {
		return false;
	}

	public void onLoop() {
		if (Saturation > 1) {
			Saturation = 1;
		}
		if (Saturation < 0)
			Saturation = 0;

		if (Value > 1)
			Value = 1;
		if (Value < 0)
			Value = 0;

		/* Convert to HSV to RGB */
		_rgb = HsvToRgb.convert(Hue, Saturation, Value);

		_rgb[0] = _averageR.process(_rgb[0]);
		_rgb[1] = _averageG.process(_rgb[1]);
		_rgb[2] = _averageB.process(_rgb[2]);

		/* Update CANifier's LED strip */
		Hardware.canifier.setLEDOutput(_rgb[0],
				CANifier.LEDChannel.LEDChannelA);
		Hardware.canifier.setLEDOutput(_rgb[1],
				CANifier.LEDChannel.LEDChannelB);
		Hardware.canifier.setLEDOutput(_rgb[2],
				CANifier.LEDChannel.LEDChannelC);
	}
}
