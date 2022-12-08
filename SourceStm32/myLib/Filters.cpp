/*
 * Filters.cpp
 *
 *  Created on: 7 gru 2022
 *      Author: Grzegorz
 */

#include <Filters.h>

#include <math.h>
#include <string.h>

//-----------------------------------------------------------------------------------------
// FiltrIR
//-----------------------------------------------------------------------------------------
FiltrIR::FiltrIR(float k) {
	firstDt = true;
	state = 0;
	mK = k;
}

void FiltrIR::inp(float x) {
	if (firstDt) {
		firstDt = false;
		state = x;
	} else {
		state = (1 - mK) * state + mK * x;
	}
}

float FiltrIR::out() {
	return state;
}

//-----------------------------------------------------------------------------------------
// FiltrFIR
//-----------------------------------------------------------------------------------------

FiltrFIR::FiltrFIR(int len) {
	memset(tab, 0, sizeof(tab));
	mOverride = false;
	mPtr = 0;
	mLen = len;
	if (mLen > MAX_LEN || mLen < 1)
		mLen = MAX_LEN;
}
void FiltrFIR::inp(float x) {
	tab[mPtr] = x;
	if (++mPtr >= MAX_LEN) {
		mPtr = 0;
		mOverride = true;
	}
}

float FiltrFIR::out() {
	float sum = 0;
	int k = 0;
	int ptr = mPtr;
	while (k < mLen) {
		if (ptr == 0) {
			if (!mOverride) {
				break;
			}
			ptr = MAX_LEN;
		}
		ptr--;
		sum += tab[ptr];
		k++;
	}
	if (k > 0)
		return sum / k;
	else
		return NAN;
}

