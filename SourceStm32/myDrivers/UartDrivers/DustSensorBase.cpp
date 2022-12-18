/*
 * DustSensorDef.cpp
 *
 *  Created on: 31 gru 2020
 *      Author: Grzegorz
 */

#include <DustSensorBase.h>

#if (DEV_DUST_INTERN)


#include <math.h>
#include <config.h>

#define TIME_DT_RD    2000
#define FILTR_FACTOR  0.8

extern Config *config;

DustSensorBase::DustSensorBase(const char *name) :
		UniDev::UniDev(name) {
	osMutexDef(DustDev);
	mDustMutex = osMutexCreate(osMutex(DustDev));

	exportDt.filterPM1_0.init(FILTR_FACTOR);
	exportDt.filterPM2_5.init(FILTR_FACTOR);
	exportDt.filterPM10.init(FILTR_FACTOR);
	exportDt.filterFormaldehyde.init(FILTR_FACTOR);
	exportDt.mLastRdDataTick = HAL_GetTick();

}

bool DustSensorBase::isDataError() {
	return (HAL_GetTick() - exportDt.mLastRdDataTick > MAX_MEAS_VALID);
}

bool DustSensorBase::isAnyConfiguredData() {
	bool q = false;
	q |= config->data.R.sensExist[ssPM1_0];
	q |= config->data.R.sensExist[ssPM2_5];
	q |= config->data.R.sensExist[ssPM10];
#if(SENSOR_CH_SO)
	if (isFormaldehyde())
		q |= config->data.R.sensExist[ssCh2o];
#endif
	return q;
}

bool DustSensorBase::getMeasValue(MeasType measType, float *val) {

	if (!isDataError())
		return false;

	if (openMutex(50)) {
		bool ret = true;
		switch (measType) {
		case ssPM1_0:
			*val = exportDt.filterPM1_0.get();
			break;
		case ssPM2_5:
			*val = exportDt.filterPM2_5.get();
			break;
		case ssPM10:
			*val = exportDt.filterPM10.get();
			break;
#if(SENSOR_CH_SO)
		case ssCh2o:
			if (isFormaldehyde())
				*val = exportDt.filterFormaldehyde.get();
			else
				ret = false;
			break;
#endif
		default:
			ret = false;
			break;
		}
		closeMutex();
		return ret;
	}
	return false;
}

bool DustSensorBase::openMutex(int tm) {
	return (osMutexWait(mDustMutex, tm) == osOK);
}
void DustSensorBase::closeMutex() {
	osMutexRelease(mDustMutex);
}

//----------------------------------------------------------------------------------------
// DustSensorNull
//----------------------------------------------------------------------------------------
DustSensorNull::DustSensorNull() :
		DustSensorBase::DustSensorBase("NullDust") {

}

void DustSensorNull::StartMeas() {
}
void DustSensorNull::StopMeas() {
}

void DustSensorNull::shell(OutStream *strm, const char *cmd) {
	strm->oMsgX(colRED, "No implemented");
}

bool DustSensorNull::isDataError() {
	return true;
}

bool DustSensorNull::isAnyConfiguredData() {
	return false;
}

void DustSensorNull::setPower(bool on) {
}
HAL_StatusTypeDef DustSensorNull::Init(SignaledClass *signObj) {
	return HAL_ERROR;
}
void DustSensorNull::tick() {
}

#endif
