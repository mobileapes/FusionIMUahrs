#include "Arduino.h"

/*
 * http://forums.ps2dev.org/viewtopic.php?p=40610&highlight=&sid=f45d5a581a18eee380aa5f81f8a909d0#40610
 */
 
float fastAtan2(float y, float x) {
	//float coeff_1 = PI/4.0f;
	float coeff_1 = 0.785398163f;
	float coeff_2 = 3.0f * coeff_1;
	float abs_y = abs(y) + 0.00000001f;	// kludge to prevent 0/0 condition;
	float angle;
	if(x >= 0.0f) {
		float r = (x - abs_y) / (x + abs_y);
		angle = coeff_1 - coeff_1 * r;
	} else {
		float r = (x + abs_y) / (abs_y - x);
		angle = coeff_2 - coeff_1 * r;
	}
	return y < 0.0f ? -angle : angle;
}

