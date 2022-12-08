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
} ShellItem; //TODO - obsolete

typedef void (*ExecCmdFun)(OutStream *strm, const char *cmd);

typedef struct {
	const char *cmd;
	const char *descr;
	ExecCmdFun fun;
} ShellItemFx;

typedef struct {
	int mnIdx;
	int idx;
} FindRes; //TODO - obsolete

extern int copyMenu(ShellItem *tab, int len, const ShellItem *src);

extern int findCmd(const ShellItem *item, const char *cmd);  //TODO - obsolete
extern void showHelp(OutStream *strm, const char *caption, const ShellItem *item); //TODO - obsolete

extern void findCmdEx(FindRes *res, const ShellItem **itemTab, const char *cmd); //TODO - obsolete
extern void showHelpEx(OutStream *strm, const char *caption, const ShellItem **itemTab); //TODO - obsolete


extern const ShellItemFx *findCmdFx(const ShellItemFx *item, const char *cmd);
extern const ShellItemFx *findCmdFxEx(const ShellItemFx **itemTab, const char *cmd);

extern void showHelpFx(OutStream *strm, const char *caption, const ShellItemFx *item);
extern void showHelpFxEx(OutStream *strm, const char *caption, const ShellItemFx **itemTab);

extern void execMenuCmd(OutStream *strm, const ShellItemFx *menu, const char *cmd, const char *menuCaption);



_END_STD_C

#endif
