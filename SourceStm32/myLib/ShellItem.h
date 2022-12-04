/*
 * MsgStream.h
 *
 *  Created on: Dec 11, 2020
 *      Author: Grzegorz
 */

#ifndef SHELLITEM_H_
#define SHELLITEM_H_

#include "_ansi.h"

#include "IoStream.h"

_BEGIN_STD_C

typedef struct {
	const char *cmd;
	const char *descr;
} ShellItem;

typedef struct{
	int mnIdx;
	int idx;
}FindRes;

extern int copyMenu(ShellItem *tab, int len,const ShellItem *src);

extern int findCmd(const ShellItem *item, const char *cmd);
extern void showHelp(OutStream *strm, const char *caption, const ShellItem *item);

extern void findCmdEx(FindRes *res, const ShellItem **itemTab, const char *cmd);
extern void showHelpEx(OutStream *strm, const char *caption, const ShellItem **itemTab);




_END_STD_C

#endif
