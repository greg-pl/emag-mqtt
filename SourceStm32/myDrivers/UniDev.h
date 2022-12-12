/*
 * UniDev.h
 *
 *  Created on: 11 gru 2022
 *      Author: Grzegorz
 */

#ifndef UNIDEV_H_
#define UNIDEV_H_

#include "Config.h"

class UniDev {
protected:
	char mName[16];
public:
	UniDev(const char *name);
	virtual bool isAnyConfiguredData() {
		return false;
	}
	virtual bool isDataError() {
		return false;
	}
	virtual bool getMeasValue(MeasType measType, float *val) {
		return false;
	}
	virtual void getDeviceStatusTxt(char *txt, int max) {
		txt[0] = 0;
	}
};

#endif /* UNIDEV_H_ */
