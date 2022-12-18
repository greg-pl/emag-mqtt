/*
 * UniDev.cpp
 *
 *  Created on: 11 gru 2022
 *      Author: Grzegorz
 */

#include <UniDev.h>
#include <string.h>
#include <math.h>
#include <UMain.h>

UniDev::UniDev(const char *name) {
	UniDevTab::AddDev(this);
	setName(name);
}

bool UniDev::isAnyConfiguredData() {
	bool q = false;
	for (int i = 0; i < SENSOR_CNT; i++) {
		if (isMeasServiced((MeasType) i)) {
			if (config->data.R.sensExist[i]){
				q = true;
				break;
			}
		}
	}
	return q;
}

void UniDev::setName(const char *name) {
	strlcpy(mName, name, sizeof(mName));
}

int UniDevTab::mDevCnt;
UniDev *UniDevTab::mDevTab[UniDevTab::UNI_DEV_MAX_CNT];

void UniDevTab::Init() {
	mDevCnt = 0;
	for (int i = 0; i < UNI_DEV_MAX_CNT; i++) {
		mDevTab[i] = NULL;
	}

}
void UniDevTab::AddDev(UniDev *dev) {
	if (mDevCnt < UNI_DEV_MAX_CNT) {
		mDevTab[mDevCnt] = dev;
		mDevCnt++;
	}

}
bool UniDevTab::getMeasValue(MeasType measType, float *val) {
	bool q = false;
	for (int i = 0; i < mDevCnt; i++) {
		bool q1 = mDevTab[i]->getMeasValue(measType, val);
		q |= q1;
		if (q1 && (!isnan(*val)))
			break;
	}
	return q;
}

