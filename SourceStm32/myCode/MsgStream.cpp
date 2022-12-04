/*
 * MsgStream.cpp
 *
 *  Created on: Dec 11, 2020
 *      Author: Grzegorz
 */

#include "string.h"
#include "MsgStream.h"

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

void showHelp(MsgStream *strm, const char *caption, const ShellItem *item) {
	const ShellItem *item2 = item;
	int mx = 0;
	while (item2->cmd) {
		int n = strlen(item2->cmd);
		mx = (mx > n) ? mx : n;
		item2++;
	}

	const char line[] = "-------------------------------------";
	if (strm->msgOpen(colCYAN)) {
		strm->msgItem(caption);
		strm->msgItem(line);
		while (item->cmd) {
			strm->msgItem("%-*s : %s", mx, item->cmd, item->descr);
			item++;
		}
		strm->msgItem(line);
		strm->msgClose();
	}
}
