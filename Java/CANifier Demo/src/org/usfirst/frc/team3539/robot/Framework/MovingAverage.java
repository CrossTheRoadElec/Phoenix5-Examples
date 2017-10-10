package org.usfirst.frc.team3539.robot.Framework;

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
		Clear();
	}

	public float Process(float input) {
		Push(input);
		return _sum / (float) _cnt;
	}

	public void Clear() {
		_in = 0;
		_ou = 0;
		_cnt = 0;

		_sum = 0;
	}

	public void Push(float d) {
		/* process it */
		_sum += d;

		/* if full, pop one */
		if (_cnt >= _cap)
			Pop();

		/* push new one */
		_d[_in] = d;
		if (++_in >= _cap)
			_in = 0;
		++_cnt;

		/* calc new min - slow */
		CalcMin();
	}

	public void Pop() {
		/* get the oldest */
		float d = _d[_ou];

		/* process it */
		_sum -= d;

		/* pop it */
		if (++_ou >= _cap)
			_ou = 0;
		--_cnt;
	}

	private void CalcMin() {
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
	public float GetSum()
	{
		return _sum;
	}
	public int GetCount()
	{
		return _cnt;
	};
	public float GetMinimum()
	{
		return _min;
	};

}
