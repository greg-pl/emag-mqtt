/*
 * ShellItem.cpp
 *
 *  Created on: Mar 23, 2021
 *      Author: Grzegorz
 */
#include "ShellItem.h"

#include "utils.h"
#include "string.h"

int copyMenu(ShellItem *tab, int len, const ShellItem *src) {
	int k = 0;
	while (tab[k].cmd != NULL && k < len - 1) {
		k++;
	}
	int m = k;
	if (src != NULL) {
		int n = 0;
		while (src[n].cmd != NULL && k < len - 1) {
			tab[k].cmd = src[n].cmd;
			tab[k].descr = src[n].descr;
			k++;
			n++;
		}
	}
	return m;
}


int findCmd(const ShellItem *item, const char *cmd) {
	int idx = 0;
	while (item->cmd) {
		if (strcmp(item->cmd, cmd) == 0) {
			return idx;
		}
		item++;
		idx++;
	}
	return -1;
}

void findCmdEx(FindRes *res, const ShellItem **itemTab, const char *cmd) {
	int mnIdx = 0;
	while (itemTab[mnIdx] != NULL) {
		int idx = findCmd(itemTab[mnIdx], cmd);
		if (idx >= 0) {
			res->mnIdx = mnIdx;
			res->idx = idx;
			return;
		}
		mnIdx++;
	}
	res->mnIdx = -1;
	res->idx = -1;
}

void showHelpEx(OutStream *strm, const char *caption, const ShellItem **itemTab) {
	const char line[] = "-------------------------------------";
	int mx = 0;

	//wyznaczenie najdłuższego polecenia
	int idx = 0;
	while (itemTab[idx] != NULL) {
		const ShellItem *item = itemTab[idx];

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

			const ShellItem *item = itemTab[idx];
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

void showHelp(OutStream *strm, const char *caption, const ShellItem *item) {
	const ShellItem *itemTab[2];
	itemTab[0] = item;
	itemTab[1] = NULL;
	showHelpEx(strm, caption, itemTab);
}


const ShellItemFx *findCmdFx(const ShellItemFx *item, const char *cmd){
	while (item->cmd) {
		if (strcmp(item->cmd, cmd) == 0) {
			return item;
		}
		item++;
	}
	return NULL;
}

const ShellItemFx *findCmdFxEx(const ShellItemFx **itemTab, const char *cmd){
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

void showHelpFx(OutStream *strm, const char *caption, const ShellItemFx *item){
	const ShellItemFx *itemTab[2];
	itemTab[0] = item;
	itemTab[1] = NULL;
	showHelpFxEx(strm, caption, itemTab);

}
void showHelpFxEx(OutStream *strm, const char *caption, const ShellItemFx **itemTab){
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


void execMenuCmd(OutStream *strm, const ShellItemFx *menu, const char *cmd, const char *menuCaption){

	char tok[20];
	const ShellItemFx *fxCmd = NULL;
	if (Token::get(&cmd, tok, sizeof(tok)))
		fxCmd = findCmdFx(menu, tok);
	if (fxCmd != NULL && fxCmd->fun != NULL) {
		fxCmd->fun(strm, cmd);
	} else {
		showHelpFx(strm, menuCaption, menu);
	}
}

