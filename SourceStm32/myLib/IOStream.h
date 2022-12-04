/*
 * MsgStream.h
 *
 *  Created on: Dec 11, 2020
 *      Author: Grzegorz
 */

#ifndef IOSTREAM_H_
#define IOSTREAM_H_

#include "stdarg.h"

typedef enum {
	colWHITE = 0, colRED, colGREEN, colBLUE, colMAGENTA, colYELLOW, colCYAN,
} TermColor;


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

class SignaledClass {
public:
	virtual void setSignal()=0;
};
/*
typedef struct {
	const char *cmd;
	const char *descr;
} ShellItem;

extern "C" void showHelp(OutStream *strm, const char *caption, const ShellItem *item);
extern "C" int findCmd(const ShellItem *item, const char *cmd);
*/
#endif /* IOSTREAM_H_ */
