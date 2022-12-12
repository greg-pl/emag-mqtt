/*
 * String.cpp
 *
 *  Created on: 25 mar 2021
 *      Author: Grzegorz
 */
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "stdarg.h"

#include <CxString.h>

CxString::CxString(int size) {
	mSize = size;
	mPStr = (char*) malloc(size + 4); // miejsce na zero
	mLen = 0;
	mPStr[0] = 0;
}

CxString::~CxString() {
	free(mPStr);
}

void CxString::setLen(int L) {
	if (L >= 0) {
		if (L > mSize) {
			L = mSize;
		}
		mLen = L;
		mPStr[mLen] = 0;
	}
}

void CxString::clear() {
	mLen = 0;
	mPStr[0] = 0;
}

void CxString::add(char ch) {
	if (mLen < mSize) {
		mPStr[mLen++] = ch;
		mPStr[mLen] = 0;
	}
}

void CxString::add(const char *str) {
	int n = strlen(str);
	if (n > mSize - mLen) {
		n = mSize - mLen;
	}
	if (n > 0) {
		memcpy(&mPStr[mLen], str, n);
		mLen += n;
		mPStr[mLen] = 0;
	}
}
void CxString::add(CxString *src) {
	add(src->p());
}

void CxString::addQuota(const char *str) {
	//todo zrobic doporządku
	add("\"");
	add(str);
	add("\"");
}

void CxString::format(const char *pFormat, ...) {
	va_list ap;
	va_start(ap, pFormat);
	mLen = vsnprintf(mPStr, mSize, pFormat, ap);
	va_end(ap);
}

void CxString::addFormat(const char *pFormat, ...) {
	int sz = mSize - mLen;
	if (sz > 0) {
		va_list ap;
		va_start(ap, pFormat);
		mLen += vsnprintf(&mPStr[mLen], sz, pFormat, ap);
		va_end(ap);
	}
}

void CxString::buildLen() {
	mLen = strlen(mPStr);
}

void CxString::fillSpaces(int end) {
	if (end > mSize)
		end = mSize;
	if (mLen < end) {
		int n = end - mLen;
		memset(&mPStr[mLen], ' ', n);
		mLen = end;
		mPStr[mLen] = 0;
	}
}
