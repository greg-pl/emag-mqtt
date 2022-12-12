/*
 * CxBuf.cpp
 *
 *  Created on: 16 kwi 2021
 *      Author: Grzegorz
 */

#include <string.h>
#include <stdlib.h>

#include <CxBuf.h>

CxBuf::CxBuf(int bgSize, int growSize) {
	mSize = bgSize;
	mGrowSz = growSize;
	mBuf = (uint8_t*) malloc(bgSize);
	mPtr = 0;

}

CxBuf::~CxBuf() {
	free(mBuf);
}

void CxBuf::replace(int idx, uint8_t val) {
	if (idx >= 0 && idx < mPtr) {
		mBuf[idx] = val;
	}
}

void CxBuf::add(const void *dt, int sz) {

	if (mPtr + sz > mSize) {
		int newSz = mSize + mGrowSz;
		if (newSz < mPtr + sz) {
			newSz = mPtr + sz + mGrowSz;
		}
		uint8_t *newBuf = (uint8_t*) malloc(newSz);
		memcpy(newBuf, mBuf, mPtr);
		free(mBuf);
		mBuf = newBuf;
		mSize = newSz;
	}
	memcpy(&mBuf[mPtr], dt, sz);
	mPtr += sz;
}

void CxBuf::addByte(uint8_t b) {
	add(&b, 1);
}

void CxBuf::addStr(const char *str, int max) {
	int n = strlen(str);
	n = (n > max) ? max : n;
	add(str, n);
	addByte(0);
}

