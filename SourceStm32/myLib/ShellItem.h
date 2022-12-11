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


typedef void (*ExecCmdFun)(OutStream *strm, const char *cmd, void *arg);

typedef struct {
	const char *cmd;
	const char *descr;
	ExecCmdFun fun;
} ShellItemFx;

extern const ShellItemFx *findCmdFx(const ShellItemFx *item, const char *cmd);
extern const ShellItemFx *findCmdFxEx(const ShellItemFx **itemTab, const char *cmd);

extern void showHelpFx(OutStream *strm, const char *caption, const ShellItemFx *item);
extern void showHelpFxEx(OutStream *strm, const char *caption, const ShellItemFx **itemTab);

extern void execMenuCmd(OutStream *strm, const ShellItemFx *menu, const char *cmd, void *arg, const char *menuCaption);
extern void execMenuCmdEx(OutStream *strm, const ShellItemFx **menuTab, const char *cmd, void *arg, const char *menuCaption);
extern void execMenuCmdArg(OutStream *strm, const ShellItemFx *menu, const char *cmd, void **tabArg, const char *menuCaption);




_END_STD_C

#endif
