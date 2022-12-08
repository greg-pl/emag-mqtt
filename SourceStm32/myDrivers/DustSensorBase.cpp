/*
 * DustSensorDef.cpp
 *
 *  Created on: 31 gru 2020
 *      Author: Grzegorz
 */

#include <DustSensorBase.h>
#include <math.h>

#define TIME_DT_RD    2000
#define FILTR_FACTOR  0.8

DustSensorBase::DustSensorBase() {
	osMutexDef(DustDev);
	mDustMutex = osMutexCreate(osMutex(DustDev));

	exportDt.filterPM1_0.init(FILTR_FACTOR);
	exportDt.filterPM2_5.init(FILTR_FACTOR);
	exportDt.filterPM10.init(FILTR_FACTOR);
	exportDt.filterFormaldehyde.init(FILTR_FACTOR);
	exportDt.mLastRdDataTick = HAL_GetTick();


}

bool DustSensorBase::isError() {
	return (HAL_GetTick() - exportDt.mLastRdDataTick > MAX_MEAS_VALID);
}

HAL_StatusTypeDef DustSensorBase::getMeas(DustMeasRec *meas) {
	HAL_StatusTypeDef st = HAL_NO_VALUE;
	if (!isError()) {
		if (openMutex(50)) {
			meas->pm1_0 = exportDt.filterPM1_0.get();
			meas->pm2_5 = exportDt.filterPM2_5.get();
			meas->pm10 = exportDt.filterPM10.get();
			meas->Formaldehyde = exportDt.filterFormaldehyde.get();
			closeMutex();
			st = HAL_OK;
		} else
			st = HAL_NO_SEMF;
	}
	if (st != HAL_OK) {
		meas->pm1_0 = NAN;
		meas->pm2_5 = NAN;
		meas->pm10 = NAN;
		meas->Formaldehyde = NAN;
	}
	return st;
}

bool DustSensorBase::openMutex(int tm) {
	return (osMutexWait(mDustMutex, tm) == osOK);
}
void DustSensorBase::closeMutex() {
	osMutexRelease(mDustMutex);
}

void DustSensorNull::StartMeas() {
}
void DustSensorNull::StopMeas() {
}

void DustSensorNull::shell(OutStream *strm, const char *cmd) {
	strm->oMsgX(colRED, "No implemented");
}

HAL_StatusTypeDef DustSensorNull::getMeas(DustMeasRec *meas) {
	meas->pm1_0 = NAN;
	meas->pm2_5 = NAN;
	meas->pm10 = NAN;
	return HAL_ERROR;
}
void DustSensorNull::setPower(bool on) {
}
HAL_StatusTypeDef DustSensorNull::Init(SignaledClass *signObj) {
	return HAL_ERROR;
}
void DustSensorNull::tick() {
}
