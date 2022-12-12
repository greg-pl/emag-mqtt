/*
 * ShellItem.cpp
 *
 *  Created on: Mar 23, 2021
 *      Author: Grzegorz
 */
#include "ShellItem.h"

#include "utils.h"
#include "string.h"
#include "Token.h"


const ShellItemFx* findCmdFx(const ShellItemFx *item, const char *cmd) {
	while (item->cmd) {
		if (strcmp(item->cmd, cmd) == 0) {
			return item;
		}
		item++;
	}
	return NULL;
}

const ShellItemFx* findCmdFxEx(const ShellItemFx **itemTab, const char *cmd) {
	int mnIdx = 0;
	while (itemTab[mnIdx] != NULL) {
		const ShellItemFx *fnd = findCmdFx(itemTab[mnIdx], cmd);
		if (fnd != NULL) {
			return fnd;
		}
		mnIdx++;
	}
	return NULL;
}

void showHelpFx(OutStream *strm, const char *caption, const ShellItemFx *item) {
	const ShellItemFx *itemTab[2];
	itemTab[0] = item;
	itemTab[1] = NULL;
	showHelpFxEx(strm, caption, itemTab);

}
void showHelpFxEx(OutStream *strm, const char *caption, const ShellItemFx **itemTab) {
	const char line[] = "-------------------------------------";
	int mx = 0;

	//wyznaczenie najdłuższego polecenia
	int idx = 0;
	while (itemTab[idx] != NULL) {
		const ShellItemFx *item = itemTab[idx];

		while (item->cmd) {
			int n = strlen(item->cmd);
			mx = (mx > n) ? mx : n;
			item++;
		}
		idx++;
	}

	if (strm->oOpen(colCYAN)) {
		strm->oMsg(caption);
		strm->oMsg(line);

		idx = 0;
		while (itemTab[idx] != NULL) {

			const ShellItemFx *item = itemTab[idx];
			while (item->cmd) {
				strm->oMsg("%-*s : %s", mx, item->cmd, item->descr);
				item++;
			}
			idx++;

		}

		strm->oMsg(line);
		strm->oClose();
	}
}

void execMenuCmd(OutStream *strm, const ShellItemFx *menu, const char *cmd, void *arg, const char *menuCaption) {
	const ShellItemFx *menuTab[2];
	menuTab[0] = menu;
	menuTab[1] = NULL;
	execMenuCmdEx(strm, menuTab, cmd, arg, menuCaption);
}

void execMenuCmdEx(OutStream *strm, const ShellItemFx **menuTab, const char *cmd, void *arg, const char *menuCaption) {

	char tok[20];
	const ShellItemFx *fxCmd = NULL;
	if (Token::get(&cmd, tok, sizeof(tok)))
		fxCmd = findCmdFxEx(menuTab, tok);
	if (fxCmd != NULL && fxCmd->fun != NULL) {
		fxCmd->fun(strm, cmd, arg);
	} else {
		showHelpFxEx(strm, menuCaption, menuTab);
	}
}

void execMenuCmdArg(OutStream *strm, const ShellItemFx *menu, const char *cmd, void **tabArg, const char *menuCaption) {
	char tok[20];
	const ShellItemFx *fxCmd = NULL;
	if (Token::get(&cmd, tok, sizeof(tok)))
		fxCmd = findCmdFx(menu, tok);
	if (fxCmd != NULL && fxCmd->fun != NULL) {
		int idx = fxCmd - menu;
		fxCmd->fun(strm, cmd, tabArg[idx]);
	} else {
		showHelpFx(strm, menuCaption, menu);
	}

}

