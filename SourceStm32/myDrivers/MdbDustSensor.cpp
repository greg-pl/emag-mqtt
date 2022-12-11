/*
 * MdbDustSensor.cpp
 *
 *  Created on: 8 gru 2022
 *      Author: Grzegorz
 */

#include "MdbDustSensor.h"

#include "utils.h"
#include "main.h"
#include "UMain.h"
#include "shell.h"
#include "Config.h"
#include "i2cDev.h"
#include "Hal.h"

#include <string.h>
#include <stdio.h>
#include <math.h>


extern ShellTask *shellTask;
extern Config *config;

//-----------------------------------------------------------------------------------------
// MdbMasterDustTask
//-----------------------------------------------------------------------------------------
MdbMasterDustTask::MdbMasterDustTask(int mdbNr, int portNr) :
		MdbMasterTask::MdbMasterTask(mdbNr, portNr) {

	memset(&dustData, 0, sizeof(dustData));
	memset(&autoRd, 0, sizeof(autoRd));
	dustData.PmStatus = 0x01; //Power_off
}

bool MdbMasterDustTask::isCfgDustOn() {
	return (config->data.R.exDev.sensExist[ssPM1_0] || config->data.R.exDev.sensExist[ssPM2_5] || config->data.R.exDev.sensExist[ssPM10]);
}

void MdbMasterDustTask::loopFunc() {
	if (isCfgDustOn()) {
		if (HAL_GetTick() - autoRd.tick > TM_AUTO_RD) {
			autoRd.tick = HAL_GetTick();
			autoRd.phase = 1;
		}
		if (autoRd.phase != 0) {
			if (state.sent.currReq == reqEMPTY) {
				switch (autoRd.phase) {
				case 1:
					autoRd.reqCnt++;
					autoRd.phase = 2;
					sendMdbFun4(reqSYS, config->data.R.rest.dustDevMdbNr, 1, 29);
					break;
				case 2:
					if (HAL_GetTick() - state.sent.tick > MAX_TIME_REPL)
						autoRd.phase = 0;
					break;
				}
			}
		}
		if (config->data.R.exDev.heater.runExternal) {
			if (autoRd.phase == 0) {
				if (isMeasValid()) {
					bool sendOrder = false;
					bool doOn = false;

					if (dustData.temperature > config->data.R.exDev.heater.tempOFF) {
						if (dustData.heaterOn) {
							sendOrder = true;
							doOn = false;
						}
					}
					if (dustData.temperature < config->data.R.exDev.heater.tempON) {
						if (!dustData.heaterOn) {
							sendOrder = true;
							doOn = true;
						}
					}

					if (sendOrder) {
						if (HAL_GetTick() - autoRd.heaterOrderLastSendTick > 5000) {
							autoRd.heaterOrderLastSendTick = HAL_GetTick();
							setHeater(reqSYS, doOn);
							if (config->data.R.exDev.heater.showMsg>=1) {
								shellTask->oMsgX(colGREEN, "MDB%u:T=%u SetHeater:%u temp=%.1f[*C]", mMdbNr, autoRd.heaterOrderLastSendTick, doOn, dustData.temperature);
							}
						}

					}
				}
			}

		}
	}

}
void MdbMasterDustTask::doOnTimeOut() {
	if (autoRd.phase > 0) {
		autoRd.phase = 0;
		shellTask->oMsgX(colRED, "MDB%u: read dust measure TIMEOUT", mMdbNr);
		strcpy(autoRd.statusTxt, "TimeOut");
	} else {
		shellTask->oMsgX(colRED, "MDB%u: TIMEOUT", mMdbNr);
	}
}

void MdbMasterDustTask::onReciveData(bool replOK, uint8_t mdbFun, const uint8_t *tab, int regCnt) {
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

void MdbMasterDustTask::setHeater(ReqSrc reqSrc, bool heaterOn) {
	uint16_t w = 0;
	if (heaterOn)
		w = HEATER_CONST_ON;
	sendMdbFun6(reqSrc, config->data.R.rest.dustDevMdbNr, 8, w);
}

void MdbMasterDustTask::showState(OutStream *strm) {
	MdbMasterTask::showState(strm);
	if (strm->oOpen(colWHITE)) {
		strm->oMsg("RdTime=%.2f[s]", (float) ((HAL_GetTick() - autoRd.redTick)) / 1000.0);
		strm->oMsg("autoRd.Status=%s", autoRd.statusTxt);
		strm->oMsg("measValid=%u", isMeasValid());
		strm->oMsg("autoRd.reqCnt=%u", autoRd.reqCnt);
		strm->oMsg("autoRd.redCnt=%u", autoRd.redCnt);
		strm->oClose();
	}

}

HAL_StatusTypeDef MdbMasterDustTask::getMeas(DustMeasRec *meas) {
	if (!isError() && isMeasValid()) {
		meas->pm1_0 = dustData.pm1_0;
		meas->pm2_5 = dustData.pm2_5;
		meas->pm10 = dustData.pm10;
		return HAL_OK;
	} else {
		meas->pm1_0 = NAN;
		meas->pm2_5 = NAN;
		meas->pm10 = NAN;
		return HAL_ERROR;
	}
}

bool MdbMasterDustTask::isMeasValid() {
	return ((dustData.PmStatus & 0x7F) == 0);
}
void MdbMasterDustTask::showMeas(OutStream *strm) {
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

void MdbMasterDustTask::funShowMeasure(OutStream *strm, const char *cmd, void *arg) {
	MdbMasterDustTask *dev = (MdbMasterDustTask*) arg;
	dev->showMeas(strm);
}

void MdbMasterDustTask::funHeaterOn(OutStream *strm, const char *cmd, void *arg) {
	MdbMasterDustTask *dev = (MdbMasterDustTask*) arg;
	dev->setHeater(reqCONSOLA, true);

}

void MdbMasterDustTask::funHeaterOff(OutStream *strm, const char *cmd, void *arg) {
	MdbMasterDustTask *dev = (MdbMasterDustTask*) arg;
	dev->setHeater(reqCONSOLA, false);
}


const ShellItemFx menuExternDustFx[] = { //
		{ "m", "pomiary",MdbMasterDustTask::funShowMeasure }, //
				{ "heater_on", "włączenie grzania", MdbMasterDustTask::funHeaterOn }, //
				{ "heater_off", "wyłączenie grzania", MdbMasterDustTask::funHeaterOff }, //

				{ NULL, NULL } };

const ShellItemFx* MdbMasterDustTask::getMenuFx(){
	return menuExternDustFx;
}


const char* MdbMasterDustTask::getMenuName() {
	return "Extern Dust Menu";
}

const char* MdbMasterDustTask::getDevName()
{
	return "dust";
}


bool MdbMasterDustTask::isError() {
	return ((autoRd.redTick == 0) || (HAL_GetTick() - autoRd.redTick > TIME_MEAS_VALID));
}
