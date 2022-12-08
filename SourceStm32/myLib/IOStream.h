/*
 * MsgStream.h
 *
 *  Created on: Dec 11, 2020
 *      Author: Grzegorz
 */

#ifndef IOSTREAM_H_
#define IOSTREAM_H_

#include "stdarg.h"
#include "EscTerminal.h"


class OutStream {
public:
	virtual void oFormatX(TermColor color, const char *pFormat, va_list ap)=0;
	virtual void oMsgX(TermColor color, const char *pFormat, ...)=0;


	virtual bool oOpen(TermColor color)= 0;
	virtual void oClose()= 0;
	virtual void oMsg(const char *pFormat, ...)= 0;
	virtual void oWr(const char *txt)=0;
	virtual void dumpBuf(TermColor color, const char *buf)=0;

};

extern "C" OutStream* getOutStream();

class SignaledClass {
public:
	virtual void setSignal()=0;
};



#endif /* IOSTREAM_H_ */
