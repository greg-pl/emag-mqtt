/*
 * MsgStream.h
 *
 *  Created on: Dec 11, 2020
 *      Author: Grzegorz
 */

#ifndef MSGSTREAM_H_
#define MSGSTREAM_H_

#include "stdarg.h"

typedef enum {
	colWHITE = 0, colRED, colGREEN, colBLUE, colMAGENTA, colYELLOW, colCYAN,
} TermColor;

typedef struct {
	const char *cmd;
	const char *descr;
} ShellItem;

class MsgStream {
public:
	virtual void msgAp(TermColor color, const char *pFormat, va_list ap)=0;
	virtual void msg(TermColor color, const char *pFormat, ...)=0;
	virtual bool msgOpen(TermColor color)= 0;
	virtual void msgClose()= 0;
	virtual void msgItem(const char *pFormat, ...)= 0;
	virtual void msgItemWr(const char *txt)=0;
	virtual void dumpBuf(TermColor color, const char *buf)=0;

};

class SignaledClass {
public:
	virtual void setSignal()=0;
};

extern "C" void showHelp(MsgStream *strm, const char *caption, const ShellItem *item);
extern "C" int findCmd(const ShellItem *item, const char *cmd);

#endif /* MSGSTREAM_H_ */
