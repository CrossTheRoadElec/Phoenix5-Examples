/*********************************************************************
 * Software License Agreement
 *
 * Copyright (C) 2016 Cross The Road Electronics.  All rights
 * reserved.
 *
 * Cross The Road Electronics (CTRE) licenses to you the right to
 * compile and modify the following source for the sole purpose of
 * expanding the feature set of the Hydro-Gear Etesia demo platform.
 * This is considered a derivative work that cannot be published,
 * distributed, sublicensed, or sold.
 *
 * CTRE does not license to you the right to use or modify in any
 * other circumstance.
 * CTRE does not license to you the right to publish,
 * distribute, sublicense, or sell copies in any circumstance.
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
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
 * (INCLUDING NEGLIGENCE), BREACH OF WARRANTY, OR OTHERWISE.
 *
 ********************************************************************/
#include "Framework/MovingAverage.h"
#include <limits>

namespace CTRE {
namespace Signals {

MovingAverage::MovingAverage(int capacity) {
	_cap = capacity;
	_d = new float[_cap];
	Clear();
}
float MovingAverage::Process(float input) {
	Push(input);
	return _sum / (float) _cnt;
}
void MovingAverage::Clear() {
	_in = 0;
	_ou = 0;
	_cnt = 0;

	_sum = 0;
}
void MovingAverage::Push(float d) {
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
void MovingAverage::Pop() {
	/* get the oldest */
	float d = _d[_ou];

	/* process it */
	_sum -= d;

	/* pop it */
	if (++_ou >= _cap)
		_ou = 0;
	--_cnt;
}
void MovingAverage::CalcMin() {
	_min = std::numeric_limits<float>::max();

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
//-------------- Properties --------------//
float MovingAverage::GetSum() {
	return _sum;
}
int MovingAverage::Count() {
	return _cnt;
}
float MovingAverage::Minimum() {
	return _min;
}
}
}
