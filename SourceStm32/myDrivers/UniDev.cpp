/*
 * UniDev.cpp
 *
 *  Created on: 11 gru 2022
 *      Author: Grzegorz
 */

#include <UniDev.h>
#include <string.h>

UniDev::UniDev(const char *name) {
	strlcpy(mName, name, sizeof(mName));
}

