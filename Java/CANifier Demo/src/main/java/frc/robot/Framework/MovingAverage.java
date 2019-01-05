/**
 * MovingAverage Class
 * 
 * Framework Class, meaning it will be added to Phoenix API in the future.
 * 
 * Class containing functions and variables related to the Moving Average of
 * a continous value/single.
 */
 package frc.robot.Framework;

public class MovingAverage {
	private int _in;
	private int _ou;
	private int _cnt;
	private int _cap;

	private float _sum;
	private float _min;

	private float[] _d;

	public MovingAverage(int capacity) {
		_cap = capacity;
		_d = new float[_cap];
		clear();
	}

	public float process(float input) {
		push(input);
		return _sum / (float) _cnt;
	}

	public void clear() {
		_in = 0;
		_ou = 0;
		_cnt = 0;

		_sum = 0;
	}

	public void push(float d) {
		/* process it */
		_sum += d;

		/* if full, pop one */
		if (_cnt >= _cap)
			pop();

		/* push new one */
		_d[_in] = d;
		if (++_in >= _cap)
			_in = 0;
		++_cnt;

		/* calc new min - slow */
		calcMin();
	}

	public void pop() {
		/* get the oldest */
		float d = _d[_ou];

		/* process it */
		_sum -= d;

		/* pop it */
		if (++_ou >= _cap)
			_ou = 0;
		--_cnt;
	}

	private void calcMin() {
		_min = Float.MAX_VALUE;

		int ou = _ou;
		int cnt = _cnt;
		while (cnt > 0) {
			float d = _d[ou];

			/* process sample */
			if (_min > d)
				_min = d;

			/* iterate */
			if (++ou >= _cnt)
				ou = 0;
			--cnt;
		}
	}

	// -------------- Properties --------------//
	public float getSum() {
		return _sum;
	}

	public int getCount() {
		return _cnt;
	}

	public float getMinimum() {
		return _min;
	}
}
