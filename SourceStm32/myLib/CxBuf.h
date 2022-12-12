/*
 * CxBuf.h
 *
 *  Created on: 16 kwi 2021
 *      Author: Grzegorz
 */

#ifndef KPHOST_CXBUF_H_
#define KPHOST_CXBUF_H_

#include "stdint.h"

class CxBuf {
private:
	int mSize;
	int mGrowSz;
	int mPtr;
	uint8_t *mBuf;
public:
	CxBuf(int bgSize, int growSize);
	virtual ~CxBuf();
	uint8_t* mem() {
		return mBuf;
	}
	void add(const void *dt, int sz);
	void addByte(uint8_t b);
	void addStr(const char *str, int max);
	void replace(int idx, uint8_t val);
	int len(){
		return mPtr;
	}

};

#endif /* KPHOST_CXBUF_H_ */
