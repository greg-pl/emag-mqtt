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
	bool isAnyConfiguredData();
	virtual bool isDataError() {
		return false;
	}
	virtual bool getMeasValue(MeasType measType, float *val) {
		return false;
	}
	virtual bool isMeasServiced(MeasType measType){
		return false;
	}

	virtual void getDeviceStatusTxt(char *txt, int max) {
		txt[0] = 0;
	}
	const char *getName(){
		return mName;
	}
	void setName(const char *name);
};

class UniDevTab {
public:
	enum {
		UNI_DEV_MAX_CNT = 10,
	};
	static int mDevCnt;
	static UniDev *mDevTab[UNI_DEV_MAX_CNT];
	static void Init();
	static void AddDev(UniDev *dev);
	static bool getMeasValue(MeasType measType, float *val);
};

#endif /* UNIDEV_H_ */
