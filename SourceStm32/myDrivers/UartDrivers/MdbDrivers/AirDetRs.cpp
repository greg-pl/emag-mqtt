/*
 * AirDetRs.cpp
 *
 *  Created on: 8 gru 2022
 *      Author: Grzegorz
 */

#include "AirDetRs.h"

#include "utils.h"
#include "UMain.h"
#include "Config.h"
#include "Hal.h"
#include "Bmp338Device.h"
#include "Sht35Device.h"

#include <string.h>
#include <stdio.h>
#include <math.h>

extern SHT35Device *sht35;
extern Bmp338Device *bmp338;
extern Config *config;

//-----------------------------------------------------------------------------------------
// MdbMasterGasTask
//-----------------------------------------------------------------------------------------
AirDetRs::AirDetRs(MdbMasterTask *mdbTask, uint8_t mdbAdr, const char *name) :
		MdbDev::MdbDev(mdbTask, mdbAdr, name) {
	memset(&autoRd, 0, sizeof(autoRd));
	memset(&gasData, 0, sizeof(gasData));

	strcpy(autoRd.statusTxt, "Start");
	zeroOfs.flag = false;
	zeroOfs.cnt = 0;
	zeroOfs.phase = 0;
	zeroOfs.sensNr = 0;
}

float AirDetRs::getGasFactor(int id) {
	float k = 1.0; //wspołczynnik temperaturowy

	float temperature1, humidity;
	sht35->getData(&temperature1, &humidity);
	float temp = temperature1;
	if (isnanf(temp)) {
		float temperature2, pressure;
		bmp338->getData(&temperature2, &pressure);
		temp = temperature2;
	}
	if (!(isnanf(temp))) {
		k = 273 / (273 + temp);
	}

	switch (id) {
	case 1: //"NO2"
		return 2.053 * k;
	case 2: //"O3"
		return 2.142 * k;
	case 3: //"CO"
		return 1.25 * k / 1000;
	case 4: //"SO2"
		return 2.859 * k;
	case 5: //"CO2"
		return 1;
	case 6: //"TEMP"
		return 1;
	case 7: //"HUMI"
		return 1;
	case 8: //"PRES"
		return 1;
	}
	return 0;
}

const char* AirDetRs::getSenName(int typ, int verTyp) {
	static char txt[20];
	switch (typ) {
	case 1:
		strcpy(txt, "NO2 ");
		break;
	case 2:
		strcpy(txt, "O3  ");
		break;
	case 3:
		strcpy(txt, "CO  ");
		break;
	case 4:
		strcpy(txt, "SO2 ");
		break;
	case 5:
		strcpy(txt, "CO2 ");
		break;
	case 6:
		strcpy(txt, "TEMP");
		break;
	case 7:
		strcpy(txt, "HUMI");
		break;
	case 8:
		strcpy(txt, "PRES");
		break;
	default:
		strcpy(txt, "??");
		break;
	}
	int n = strlen(txt);
	sprintf(&txt[n], "(v%d)", verTyp);
	return txt;
}

void AirDetRs::onReciveData(bool replOK, uint8_t mdbFun, const uint8_t *tab, int regCnt) {
	if (autoRd.phase > 0) {
		if (replOK) {

			switch (autoRd.phase) {
			case 2: {
				uint16_t w = GetWord(&tab[0]);
				gasData.devCnt = w & 0xff;
				gasData.devCntTh = (gasData.devCnt < MAX_DEV_CNT) ? gasData.devCnt : MAX_DEV_CNT;

				gasData.serialNum = GetWord(&tab[2]);
				gasData.ProdYear = GetWord(&tab[4]);
				gasData.FirmwareVer = GetWord(&tab[6]);
				gasData.FailureCode = GetWord(&tab[8]);
				autoRd.phase = 3;
			}
				break;
			case 4: {
				autoRd.redTick = HAL_GetTick();
				autoRd.redCnt++;
				strcpy(autoRd.statusTxt, "OK");
				for (int i = 0; i < gasData.devCntTh; i++) {
					int base = 8 * i;
					uint16_t id = GetWord(&tab[base + 0]);

					SensorData *sensor = &gasData.sensorTab[i];

					sensor->SensorType = id >> 8;
					sensor->VerTyp = id & 0xff;
					sensor->Status = GetWord(&tab[base + 2]);
					int v = GetWord(&tab[base + 4]);
					int vh = GetWord(&tab[base + 6]);
					v = v | (vh << 16);
					sensor->valueHd = v;
					float factor = getGasFactor(sensor->SensorType);
					sensor->valueFiz = factor * v;
					if (sensor->filtrIR == NULL)
						sensor->filtrIR = new FiltrIR(config->data.R.exDev.filtrIRConst);
					if (sensor->filtrFIR == NULL)
						sensor->filtrFIR = new FiltrFIR(config->data.R.exDev.filtrFIRLength);
					sensor->filtrIR->inp(sensor->valueFiz);
					sensor->filtrFIR->inp(sensor->valueFiz);
				}
				autoRd.phase = 0;
			}
				break;

			}
		} else {
			autoRd.phase = 0;
		}
	}
	if (zeroOfs.phase > 0) {
		getOutStream()->oMsgX(colWHITE, "MDB%u: Ack %s. Phase=%u sensNr=%u", getMdbNr(), ErrOk(!replOK), zeroOfs.phase, zeroOfs.sensNr);
		if (replOK) {
			switch (zeroOfs.phase) {
			case 2:
				zeroOfs.sensNr++;
				if (zeroOfs.sensNr < gasData.devCntTh) {
					sendZeroOfs_Phase1Frame();
				} else
					zeroOfs.phase++;
				break;
			case 4:
				zeroOfs.phase++;
				break;
			case 6:
				zeroOfs.sensNr++;
				if (zeroOfs.sensNr < gasData.devCntTh) {
					sendZeroOfs_Phase5Frame();
				} else {
					getOutStream()->oMsgX(colGREEN, "MDB%u: Send zero offset OK", getMdbNr());
					zeroOfs.phase = 0;
				}
				break;
			}
		} else {
			getOutStream()->oMsgX(colRED, "MDB%u: Send zero offset ERROR. Phase=%u sensNr=%u", getMdbNr(), zeroOfs.phase, zeroOfs.sensNr);
			zeroOfs.phase = 0;
		}
	}
}

void AirDetRs::onTimeOut() {
	if (zeroOfs.phase > 0) {
		zeroOfs.phase = 0;
		getOutStream()->oMsgX(colRED, "MDB%u: Send zero offset TIMEOUT. Phase=%u sensNr=%u", getMdbNr(), zeroOfs.phase, zeroOfs.sensNr);
	} else if (autoRd.phase > 0) {
		autoRd.phase = 0;
		getOutStream()->oMsgX(colRED, "MDB%u: read measure TIMEOUT", getMdbNr());
		strcpy(autoRd.statusTxt, "TimeOut");
	} else {
		getOutStream()->oMsgX(colRED, "MDB%u: TIMEOUT", getMdbNr());
	}
}

void AirDetRs::sendZeroOfs_FrameZero(uint16_t *tab) {
	tab[0] = 0x30c6;
	tab[1] = 0x431b;
	tab[2] = 0x0000;
	tab[3] = 0x0000;
	tab[4] = 0x0000;
	tab[5] = 0x0000;
	tab[6] = 0x0000;
}

void AirDetRs::sendZeroOfs_Phase1Frame() {
	uint16_t tab[7];

	sendZeroOfs_FrameZero(tab);
	tab[6] = zeroOfs.sensNr * 0x100;
	sendMdbFun16(reqSYS, mMdbAdr, 10001, 7, tab);
}

void AirDetRs::sendZeroOfs_Phase5Frame() {
	uint16_t tab[7];

	sendZeroOfs_FrameZero(tab);
	tab[6] = zeroOfs.sensNr * 0x100 + 3;
	sendMdbFun16(reqSYS, mMdbAdr, 10001, 7, tab);
}

void AirDetRs::sendZeroOfs_Phase3Frame() {
	uint16_t tab[7];

	sendZeroOfs_FrameZero(tab);
	for (int i = 0; i < gasData.devCntTh; i++) {
		tab[2 + i] = zeroOfs.tab[i];
	}
	tab[6] = 0x0005;
	sendMdbFun16(reqSYS, mMdbAdr, 10001, 7, tab);
}

void AirDetRs::loopFunc() {
	if (isAnyConfiguredData()) {
		if (HAL_GetTick() - autoRd.tick > TM_AUTO_RD) {
			if (zeroOfs.phase == 0) {
				autoRd.tick = HAL_GetTick();
				autoRd.phase = 1;
			}
		}
		if (autoRd.phase != 0) {
			if (!isCurrenReq()) {
				switch (autoRd.phase) {
				case 1:
					autoRd.reqCnt++;
					autoRd.phase = 2;
					sendMdbFun4(reqSYS, mMdbAdr, 1, 4); //todo było 1,5
					break;
				case 3:
					sendMdbFun4(reqSYS, mMdbAdr, 1001, 4 * gasData.devCntTh);
					autoRd.phase = 4;
					break;
				case 4:
				case 2:
					if (HAL_GetTick() - getSentTick() > MAX_TIME_REPL)
						autoRd.phase = 0;
					break;
				}
			}
		}
		if (zeroOfs.flag) {
			if (autoRd.phase == 0) {
				zeroOfs.flag = false;
				zeroOfs.phase = 1;
				zeroOfs.sensNr = 0;
				getOutStream()->oMsgX(colYELLOW, "ZeroOfs  cnt=%u", zeroOfs.cnt);
			}
		}
		if (zeroOfs.phase != 0) {
			if (!isCurrenReq()) {
				switch (zeroOfs.phase) {
				case 1:
					zeroOfs.phase = 2;
					zeroOfs.sensNr = 0;
					sendZeroOfs_Phase1Frame();
					break;
				case 3:
					zeroOfs.phase = 4;
					zeroOfs.sensNr = 0;
					sendZeroOfs_Phase3Frame();
					break;
				case 5:
					zeroOfs.phase = 6;
					zeroOfs.sensNr = 0;
					sendZeroOfs_Phase5Frame();
					break;
				case 2:
				case 4:
				case 6:
					if (HAL_GetTick() - getSentTick() > MAX_TIME_REPL + 500)
						zeroOfs.phase = 0;
					break;
				}

			}
		}
	}
}

void AirDetRs::showState(OutStream *strm) {
	if (strm->oOpen(colWHITE)) {
		strm->oMsg("RedCnt=%d", autoRd.redCnt);
		strm->oMsg("TmRd=%.2f[s]", (HAL_GetTick() - autoRd.redTick) / 1000.0);
		strm->oClose();

	}
}

bool AirDetRs::isMeasValid(uint16_t status) {
	return ((status & 0x07) == 0);
}

const char* AirDetRs::getSensValidStr(uint16_t status) {
	if ((status & 0x07) != 0)
		return "ERR";
	else if ((status & 0x3FF) != 0)
		return "WARN";
	else
		return "OK";
}

void AirDetRs::showMeas(OutStream *strm) {
	if (strm->oOpen(colWHITE)) {
		strm->oMsg("RedCnt=%d", autoRd.redCnt);
		strm->oMsg("TmRd=%.2f[s]", (HAL_GetTick() - autoRd.redTick) / 1000.0);
		strm->oMsg("MeasCnt=%d (%d)", gasData.devCnt, gasData.devCntTh);
		strm->oMsg("serialNum=%d", gasData.serialNum);
		strm->oMsg("ProdYear=%d", gasData.ProdYear);
		strm->oMsg("FirmwareVer=%d.%03d", gasData.FirmwareVer >> 8, gasData.FirmwareVer & 0xff);
		strm->oMsg("FailureCode=%d", gasData.FailureCode);

		SensorData *pD = gasData.sensorTab;
		for (int i = 0; i < gasData.devCntTh; i++) {
			int n = snprintf(gtxt, sizeof(gtxt), "%u. %s %s Status=0x%04X ", i + 1, //
			getSenName(pD->SensorType, pD->VerTyp), getSensValidStr(pD->Status), pD->Status);

			if (isMeasValid(pD->Status)) {
				n += snprintf(&gtxt[n], sizeof(gtxt) - n, "v=%d[ppb]  %.3f[ug/m3]", pD->valueHd, pD->valueFiz);
				if (pD->filtrFIR != NULL) {
					n += snprintf(&gtxt[n], sizeof(gtxt) - n, " (FIR:%.3f) (IR:%.3f)", pD->filtrFIR->out(), pD->filtrIR->out());
				}
			}
			strm->oMsg(gtxt);
			pD++;

		}

		strm->oClose();
	}
}

bool AirDetRs::isDataError() {
	return ((autoRd.redTick == 0) || (HAL_GetTick() - autoRd.redTick > TIME_MEAS_VALID));
}

bool AirDetRs::isAnyConfiguredData() {
	if (config->data.R.exDev.sensExist[ssCO])
		return true;
	if (config->data.R.exDev.sensExist[ssSO2])
		return true;
	if (config->data.R.exDev.sensExist[ssO3])
		return true;
	if (config->data.R.exDev.sensExist[ssNO2])
		return true;
	return false;
}


bool AirDetRs::getMeasValue(MeasType measType, float *val) {

	if (isDataError()) {
		*val = NAN;
		return false;
	}

	int filtrType = config->data.R.exDev.gasFiltrType;

	int gasCode = 0;
	switch (measType) {
	case ssNO2:
		gasCode = 1;
		break;
	case ssO3:
		gasCode = 2;
		break;
	case ssCO:
		gasCode = 3;
		break;
	case ssSO2:
		gasCode = 4;
		break;
	default:
		*val = NAN;
		return false;
	}

	for (int i = 0; i < gasData.devCntTh; i++) {
		int code = gasData.sensorTab[i].SensorType;
		if (code == gasCode) {
			if (isMeasValid(gasData.sensorTab[i].Status)) {
				float v = gasData.sensorTab[i].valueFiz;
				switch (filtrType) {
				case 1:
					if (gasData.sensorTab[i].filtrFIR != NULL)
						v = gasData.sensorTab[i].filtrFIR->out();
					break;
				case 2:
					if (gasData.sensorTab[i].filtrIR != NULL)
						v = gasData.sensorTab[i].filtrIR->out();
					break;
				}
				*val = v;
			} else {
				*val = NAN;
			}
			return true;
		}
	}
	*val = NAN;
	return false;
}

bool AirDetRs::zeroGasFromSMS(const char *ptr, char *resText, int maxLen) {
	zeroOfs.cnt = 0;
	while (zeroOfs.cnt < MAX_DEV_CNT) {
		int a;
		if (Token::getAsInt(&ptr, &a)) {
			zeroOfs.tab[zeroOfs.cnt++] = a;
		} else
			break;
	}
	if (zeroOfs.cnt == gasData.devCntTh) {
		if (getOutStream()->oOpen(colBLUE)) {
			getOutStream()->oMsg("GAS: Zero offset z SMS");
			for (int i = 0; i < gasData.devCntTh; i++) {
				getOutStream()->oMsg("%u. z=%d", i + 1, (int) ((int16_t) zeroOfs.tab[i]));
			}
			getOutStream()->oClose();
		}

		zeroOfs.flag = true;
		strlcpy(resText, "ZERO-GAS. Procedura rozpoczęta.", maxLen);
		return true;
	} else {
		getOutStream()->oMsgX(colBLUE, "SMS Błąd. Ilość parametrów powinna być %u", gasData.devCntTh);
		snprintf(resText, maxLen, "ZERO-GAS. Błąd, ilość parametrów powinna być %u", gasData.devCntTh);
		return false;
	}
}

void AirDetRs::getDeviceStatusTxt(char *txt, int max) {
	snprintf(txt, max, "ReqCnt=%u RdCnt=%u TimeOutCnt=%u ST=%s", autoRd.reqCnt, autoRd.redCnt, mMdb->getTimeOutCnt(), autoRd.statusTxt);
}

void AirDetRs::funShowMeasure(OutStream *strm, const char *cmd, void *arg) {
	AirDetRs *gas = (AirDetRs*) arg;
	gas->showMeas(strm);
}
void AirDetRs::funSetZero(OutStream *strm, const char *cmd, void *arg) {
	AirDetRs *gas = (AirDetRs*) arg;

	strm->oMsgX(colWHITE, "Zero offset");
	gas->zeroOfs.cnt = 0;
	while (gas->zeroOfs.cnt < MAX_DEV_CNT) {
		int a;
		if (Token::getAsInt(&cmd, &a)) {
			gas->zeroOfs.tab[gas->zeroOfs.cnt++] = a;
		} else
			break;
	}
	if (gas->zeroOfs.cnt == gas->gasData.devCntTh) {
		gas->zeroOfs.flag = true;
	} else {
		strm->oMsgX(colYELLOW, "Błąd. Ilość parametrów powinna być %u", gas->gasData.devCntTh);
	}

}

const ShellItemFx menuGasFx[] = { //
		{ "m", "pomiary", AirDetRs::funShowMeasure }, //
				{ "zero", "ustawienie zera", AirDetRs::funSetZero }, //

				{ NULL, NULL } };

void AirDetRs::shell(OutStream *strm, const char *cmd) {
	execMenuCmd(strm, menuGasFx, cmd, this, "GAS sensor Menu");
}

