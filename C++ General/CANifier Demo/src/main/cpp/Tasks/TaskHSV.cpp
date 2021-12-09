#include "Tasks/TaskHSV.h"
#include "ctre/Phoenix.h"
#include "Platform/Platform.h"


static void hsvtorgb(double hDegrees, double S, double V, float* r, float* g,
		float* b) {
	double R, G, B;
	double H = hDegrees;

	//Handles wrap-around
	if (H < 0) {
		H += 360;
	};
	if (H >= 360) {
		H -= 360;
	};

	if (V <= 0)
		R = G = B = 0;
	else if (S <= 0)
		R = G = B = V;
	else {
		double hf = H / 60.0;
		int i = (int) floor(hf);
		double f = hf - i;
		double pv = V * (1 - S);
		double qv = V * (1 - S * f);
		double tv = V * (1 - S * (1 - f));
		switch (i) {
		//Red is dominant color
		case 0:
			R = V;
			G = tv;
			B = pv;
			break;

			//Green is dominant color
		case 1:
			R = qv;
			G = V;
			B = pv;
			break;
		case 2:
			R = pv;
			G = V;
			B = tv;
			break;

			//Blue is dominant color
		case 3:
			R = pv;
			G = qv;
			B = V;
			break;
		case 4:
			R = tv;
			G = pv;
			B = V;
			break;

			//Red is dominant color
		case 5:
			R = V;
			G = pv;
			B = qv;
			break;

			//Back-up case statements, in case our math is wrong
		case 6:
			R = V;
			G = tv;
			B = pv;
			break;
		case -1:
			R = V;
			G = pv;
			B = qv;
			break;

			//Color is not defined
		default:
			//pretend color is black and white
			R = G = B = V;
			break;
		}
	}
	*r = (float) R;
	*g = (float) G;
	*b = (float) B;
}

/* Destructor */
TaskHSV::~TaskHSV() { }

/* ILoopable */
void TaskHSV::OnStart() {
}
void TaskHSV::OnStop() {
}
bool TaskHSV::IsDone() {
	return false;
}
void TaskHSV::OnLoop() {
	if (Saturation > 1) {
		Saturation = 1;
	}
	if (Saturation < 0)
		Saturation = 0;

	if (Value > 1)
		Value = 1;
	if (Value < 0)
		Value = 0;

	/* Convert to RGB */
	hsvtorgb(Hue, Saturation, Value, &_r, &_g, &_b);

	_r = _averageR->Process(_r);
	_g = _averageG->Process(_g);
	_b = _averageB->Process(_b);

	/* Update CANifier's LED strip */
	Hardware::canifier->SetLEDOutput(_r, CANifier::LEDChannel::LEDChannelA);
	Hardware::canifier->SetLEDOutput(_g, CANifier::LEDChannel::LEDChannelB);
	Hardware::canifier->SetLEDOutput(_b, CANifier::LEDChannel::LEDChannelC);
}
