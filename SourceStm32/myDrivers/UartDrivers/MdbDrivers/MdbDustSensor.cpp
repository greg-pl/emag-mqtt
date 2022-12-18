/*
 * MdbDustSensor.cpp
 *
 *  Created on: 8 gru 2022
 *      Author: Grzegorz
 */

#include "MdbDustSensor.h"

#if (DEV_DUST_MDB)

#include "utils.h"
#include "main.h"
#include "UMain.h"
#include "Config.h"
#include "i2cDev.h"
#include "Hal.h"

#include <string.h>
#include <stdio.h>
#include <math.h>

extern Config *config;

//-----------------------------------------------------------------------------------------
// MdbMasterDustTask
//-----------------------------------------------------------------------------------------
ExtDustsensor::ExtDustsensor(MdbMasterTask *mdbTask, uint8_t mdbAdr, const char *name) :
		MdbDev::MdbDev(mdbTask, mdbAdr, name) {

	memset(&dustData, 0, sizeof(dustData));
	memset(&autoRd, 0, sizeof(autoRd));
	dustData.PmStatus = 0x01; //Power_off
}

void ExtDustsensor::loopFunc() {
	if (isAnyConfiguredData()) {
		if (HAL_GetTick() - autoRd.tick > TM_AUTO_RD) {
			autoRd.tick = HAL_GetTick();
			autoRd.phase = 1;
		}
		if (autoRd.phase != 0) {
			if (!isCurrenReq()) {
				switch (autoRd.phase) {
				case 1:
					autoRd.reqCnt++;
					autoRd.phase = 2;
					sendMdbFun4(reqSYS, config->data.R.rest.dustDevMdbNr, 1, 29);
					break;
				case 2:
					if (HAL_GetTick() - getSentTick() > MAX_TIME_REPL)
						autoRd.phase = 0;
					break;
				}
			}
		}

#if (HEATER)
		if (config->data.R.heater.runExternal) {
			if (autoRd.phase == 0) {
				if (isMeasValid()) {
					bool sendOrder = false;
					bool doOn = false;

					if (dustData.temperature > config->data.R.heater.tempOFF) {
						if (dustData.heaterOn) {
							sendOrder = true;
							doOn = false;
						}
					}
					if (dustData.temperature < config->data.R.heater.tempON) {
						if (!dustData.heaterOn) {
							sendOrder = true;
							doOn = true;
						}
					}

					if (sendOrder) {
						if (HAL_GetTick() - autoRd.heaterOrderLastSendTick > 5000) {
							autoRd.heaterOrderLastSendTick = HAL_GetTick();
							setHeater(reqSYS, doOn);
							if (config->data.R.heater.showMsg >= 1) {
								getOutStream()->oMsgX(colGREEN, "MDB%u:T=%u SetHeater:%u temp=%.1f[*C]", getMdbNr(), autoRd.heaterOrderLastSendTick, doOn, dustData.temperature);
							}
						}
					}
				}
			}

		}
#endif
	}

}
void ExtDustsensor::onTimeOut() {
	if (autoRd.phase > 0) {
		autoRd.phase = 0;
		getOutStream()->oMsgX(colRED, "MDB%u: read dust measure TIMEOUT", getMdbNr());
		strcpy(autoRd.statusTxt, "TimeOut");
	} else {
		getOutStream()->oMsgX(colRED, "MDB%u: TIMEOUT", getMdbNr());
	}
}

void ExtDustsensor::onReciveData(bool replOK, uint8_t mdbFun, const uint8_t *tab, int regCnt) {
	if (autoRd.phase > 0) {
		if (replOK) {

			switch (autoRd.phase) {
			case 2: {
				dustData.devID = GetWord(&tab[0]);
				dustData.serialNumer = GetWord(&tab[2]);
				dustData.productYear = GetWord(&tab[4]);
				uint16_t w = GetWord(&tab[6]);
				dustData.firmware.ver = w >> 8;
				dustData.firmware.rev = w & 0xff;

				dustData.failureCode = GetWord(&tab[8]);
				dustData.temperature = GetFloat(&tab[10]);
				dustData.heaterOn = (GetWord(&tab[14]) == HEATER_CONST_ON);
				dustData.PmStatus = GetWord(&tab[16]);
				dustData.pm1_0 = GetFloat(&tab[18]);
				dustData.pm2_5 = GetFloat(&tab[22]);
				dustData.pm4_0 = GetFloat(&tab[26]);
				dustData.pm10 = GetFloat(&tab[30]);
				dustData.dustCnt0_5 = GetFloat(&tab[34]);
				dustData.dustCnt1_0 = GetFloat(&tab[38]);
				dustData.dustCnt2_5 = GetFloat(&tab[42]);
				dustData.dustCnt4_0 = GetFloat(&tab[46]);
				dustData.dustCnt10 = GetFloat(&tab[50]);
				dustData.dustSize = GetFloat(&tab[54]);

				autoRd.phase = 0;
				autoRd.redCnt++;
				autoRd.redTick = HAL_GetTick();
				strcpy(autoRd.statusTxt, "OK");

			}
				break;

			}
		} else {
			autoRd.phase = 0;
		}
	}
}

void ExtDustsensor::setHeater(ReqSrc reqSrc, bool heaterOn) {
	uint16_t w = 0;
	if (heaterOn)
		w = HEATER_CONST_ON;
	sendMdbFun6(reqSrc, config->data.R.rest.dustDevMdbNr, 8, w);
}

void ExtDustsensor::showState(OutStream *strm) {
	if (strm->oOpen(colWHITE)) {
		strm->oMsg("RdTime=%.2f[s]", (float) ((HAL_GetTick() - autoRd.redTick)) / 1000.0);
		strm->oMsg("autoRd.Status=%s", autoRd.statusTxt);
		strm->oMsg("measValid=%u", isMeasValid());
		strm->oMsg("autoRd.reqCnt=%u", autoRd.reqCnt);
		strm->oMsg("autoRd.redCnt=%u", autoRd.redCnt);
		strm->oClose();
	}

}


bool ExtDustsensor::isMeasValid() {
	return ((dustData.PmStatus & 0x7F) == 0);
}

bool ExtDustsensor::isAnyConfiguredData() {
	return (config->data.R.sensExist[ssPM1_0] || config->data.R.sensExist[ssPM2_5] || config->data.R.sensExist[ssPM10]);
}

bool ExtDustsensor::isDataError() {
	return ((autoRd.redTick == 0) || (HAL_GetTick() - autoRd.redTick > TIME_MEAS_VALID));
}


bool ExtDustsensor::getMeasValue(MeasType measType, float *val) {
	if (!isDataError() && isMeasValid()) {

		switch (measType) {
		case ssPM1_0:
			*val = dustData.pm1_0;
			return true;
		case ssPM2_5:
			*val = dustData.pm2_5;
			return true;
		case ssPM10:
			*val = dustData.pm10;
			return true;
		default:
			return false;
		}
	}
	return false;
}

void ExtDustsensor::showMeas(OutStream *strm) {
	if (strm->oOpen(colWHITE)) {
		strm->oMsg("DevId=0x%04X", dustData.devID);
		strm->oMsg("SerNum=%u", dustData.serialNumer);
		strm->oMsg("prodYear=%u", dustData.productYear);
		strm->oMsg("FirmVer=%u.%03u", dustData.firmware.ver, dustData.firmware.rev);
		strm->oMsg("FailureCode=0x%04X", dustData.failureCode);
		strm->oMsg("Temperature=%.1f", dustData.temperature);
		strm->oMsg("HeaterOn=%u", dustData.heaterOn);
		strm->oMsg("PM_Status=0x%04X", dustData.PmStatus);
		if (isMeasValid()) {
			strm->oMsg("PM 1.0= %.1f[ug/m3]", dustData.pm1_0);
			strm->oMsg("PM 2.5= %.1f[ug/m3]", dustData.pm2_5);
			strm->oMsg("PM 4.0= %.1f[ug/m3]", dustData.pm4_0);
			strm->oMsg("PM 10 = %.1f[ug/m3]", dustData.pm10);
			strm->oMsg("DustCnt'0_5'=%.2f[#/cm3]", dustData.dustCnt0_5);
			strm->oMsg("DustCnt'1_0'=%.2f[#/cm3]", dustData.dustCnt1_0);
			strm->oMsg("DustCnt'2_5'=%.2f[#/cm3]", dustData.dustCnt2_5);
			strm->oMsg("DustCnt'4_0'=%.2f[#/cm3]", dustData.dustCnt4_0);
			strm->oMsg("DustCnt'10' =%.2f[#/cm3]", dustData.dustCnt10);
			strm->oMsg("DustSize =%.2f[nm]", dustData.dustSize);
		}
		strm->oClose();
	}
}

void ExtDustsensor::funShowMeasure(OutStream *strm, const char *cmd, void *arg) {
	ExtDustsensor *dev = (ExtDustsensor*) arg;
	dev->showMeas(strm);
}

void ExtDustsensor::funHeaterOn(OutStream *strm, const char *cmd, void *arg) {
	ExtDustsensor *dev = (ExtDustsensor*) arg;
	dev->setHeater(reqCONSOLA, true);

}

void ExtDustsensor::funHeaterOff(OutStream *strm, const char *cmd, void *arg) {
	ExtDustsensor *dev = (ExtDustsensor*) arg;
	dev->setHeater(reqCONSOLA, false);
}

const ShellItemFx menuExternDustFx[] = { //
		{ "m", "pomiary", ExtDustsensor::funShowMeasure }, //
				{ "heater_on", "włączenie grzania", ExtDustsensor::funHeaterOn }, //
				{ "heater_off", "wyłączenie grzania", ExtDustsensor::funHeaterOff }, //

				{ NULL, NULL } };

void ExtDustsensor::shell(OutStream *strm, const char *cmd) {
	execMenuCmd(strm, menuExternDustFx, cmd, this, "Extern Dust Menu");
}

#endif

