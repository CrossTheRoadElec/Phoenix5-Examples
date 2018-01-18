#pragma once

namespace CTRE {
namespace Signals {

class MovingAverage {
public:
	MovingAverage(int capacity);
	float Process(float input);
	void Clear();
	void Push(float d);
	void Pop();

	/* Properties */
	float GetSum();
	int Count();
	float Minimum();
private:
	int _in;
	int _ou;
	int _cnt;
	int _cap;
	float _sum;
	float _min;
	float* _d;

	void CalcMin();
};
}
}
