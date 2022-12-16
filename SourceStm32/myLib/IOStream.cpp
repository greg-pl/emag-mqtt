/*
 * IOStream.cpp
 *
 *  Created on: Jun 14, 2021
 *      Author: Grzegorz
 */

#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include "myDef.h"

#include "IOStream.h"
#include "utils.h"

OutStream::OutStream() :
		OutHdStream::OutHdStream() {

}

extern "C" byte getCurrentLanguage();

void OutStream::oFormatX(TermColor color, const char *pFormat, va_list ap) {
	if (oOpen(color)) {
		int len = vsnprintf(outBuf, sizeof(outBuf), pFormat, ap);
		putOut(outBuf, len);
		putStr("\r\n");
		oClose();
	}
}

void OutStream::oMsgX(TermColor color, const char *pFormat, ...) {
	va_list ap;
	va_start(ap, pFormat);
	oFormatX(color, pFormat, ap);
	va_end(ap);
}

bool OutStream::oOpen(TermColor color) {
	putStr(TERM_CLEAR_LINE);
	putStr(EscTerminal::getColorStr(color));
	return true;
}

void OutStream::oSetColor(TermColor color) {
	putStr(EscTerminal::getColorStr(color));
}

void OutStream::oClose() {
	escTermShowLineNoMx();
}

void OutStream::oBuf(const void *mem, int len) {
	putOut(mem, len);
	putStr("\r\n");
}

void OutStream::oWr(const char *txt) {
	int len = strlen(txt);
	putOut(txt, len);
	putStr("\r\n");
}

void OutStream::oMsg(const char *pFormat, ...) {
	va_list ap;
	va_start(ap, pFormat);
	int len = vsnprintf(outBuf, sizeof(outBuf), pFormat, ap);
	va_end(ap);
	putOut(outBuf, len);
	putStr("\r\n");
}

void OutStream::oMsgNN(const char *pFormat, ...) {
	va_list ap;
	va_start(ap, pFormat);
	int len = vsnprintf(outBuf, sizeof(outBuf), pFormat, ap);
	va_end(ap);
	putOut(outBuf, len);
}

void OutStream::oBufX(TermColor color, const void *buf, int len) {
	if (oOpen(color)) {
		putOut(buf, len);
		oClose();
	}

}

void OutStream::oWrX(TermColor color, const char *buf) {
	if (oOpen(color)) {
		int len = strlen(buf);
		putOut(buf, len);
		oClose();
	}
}

void OutStream::oBinBuf(const void *buf, int len) {
	char tt[10];
	const char *txt = (const char*) buf;
	for (int i = 0; i < len; i++) {
		char ch = txt[i];
		if (ch >= ' ' && ch < 0x80) {
			putOut(&ch, 1);
		} else if (ch == '\n') {
			strcpy(tt, "\\n");
			putOut(tt, strlen(tt));
		} else if (ch == '\r') {
			strcpy(tt, "\\r");
			putOut(tt, strlen(tt));
		} else {
			snprintf(tt, sizeof(tt), "\\0x%02X", ch);
			putOut(tt, strlen(tt));
		}
	}
}

void OutStream::oBinBufX(TermColor color, const void *buf, int len) {
	if (oOpen(color)) {
		oBinBuf(buf, len);
		putStr("\r\n");
		oClose();
	}

}

void OutStream::oBinBufHex(const void *buf, int len) {
	char tt[10];
	const char *txt = (const char*) buf;
	for (int i = 0; i < len; i++) {
		snprintf(tt, sizeof(tt), "%02X ", txt[i]);
		int n = 3;
		if (i == len - 1)
			n--;
		putOut(tt, n);
	}

}

void OutStream::oBinBufHexX(TermColor color, const void *buf, int len) {
	if (oOpen(color)) {
		oBinBufHex(buf, len);
		putStr("\r\n");
		oClose();
	}
}

