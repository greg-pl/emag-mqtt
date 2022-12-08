/*
 * GasS873.cpp
 *
 *  Created on: 7 gru 2022
 *      Author: Grzegorz
 */

#include "GasS873.h"

#include <IOStream.h>
#include "TaskClass.h"
#include "uart.h"
#include "GlobData.h"
//#include "DustSensorBase.h"
#include "ShellItem.h"
#include "Config.h"
#include "shell.h"

#include "math.h"
#include "string.h"
#include "stdio.h"

extern ShellTask *shellTask;
extern Config *config;

//-----------------------------------------------------------------------------------------
// GasS873
//-----------------------------------------------------------------------------------------
GasS873::GasS873(int mdbNr, int portNr) :
		MdbMasterTask::MdbMasterTask(mdbNr, portNr) {
	memset(&autoRd, 0, sizeof(autoRd));
	memset(&gasData, 0, sizeof(gasData));
	strcpy(autoRd.statusTxt, "Start");
}

bool GasS873::isAnyConfiguredData() {
	if (config->data.R.exDev.sensExist[ssCO])
		return true;
	if (config->data.R.exDev.sensExist[ssCO2])
		return true;
	if (config->data.R.exDev.sensExist[ssTEMPERATURE])
		return true;
	if (config->data.R.exDev.sensExist[ssHUMIDITY])
		return true;
	if (config->data.R.exDev.sensExist[ssPRESSURE])
		return true;
	return false;
}

bool GasS873::isDataError() {
	return ((autoRd.redTick == 0) || (HAL_GetTick() - autoRd.redTick > TIME_MEAS_VALID));
}

MeasType GasS873::getMeasType(uint8_t sensorType, uint8_t verTyp) {
	if (sensorType == 0x03 && verTyp == 0x03)
		return ssCO;
	if (sensorType == 0x05 && verTyp == 0x04)
		return ssCO2;
	if (sensorType == 0x06 && verTyp == 0x05)
		return ssTEMPERATURE;
	if (sensorType == 0x07 && verTyp == 0x05)
		return ssHUMIDITY;
	if (sensorType == 0x08 && verTyp == 0x06)
		return ssPRESSURE;
	return ssUNKNOWN;
}

float GasS873::getMeasFactor(MeasType mType) {
	switch (mType) {
	case ssCO:
		return 1; //ppb
	case ssCO2:
		return 1; //ppm
	case ssTEMPERATURE:
		return 0.01; //*C
	case ssHUMIDITY:
		return 0.01; //%RH
	case ssPRESSURE:
		return 0.01; //Pa
	default:
		return 1;
	}
}

void GasS873::onReciveData(bool replOK, uint8_t mdbFun, const uint8_t *tab, int regCnt) {
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
					S873Data *sensor = &gasData.sensorTab[i];

					uint16_t id = GetWord(&tab[base + 0]);

					sensor->SensorType = id >> 8;
					sensor->VerTyp = id & 0xff;
					sensor->measType = getMeasType(sensor->SensorType, sensor->VerTyp);

					sensor->Status = GetWord(&tab[base + 2]);
					int vl = GetWord(&tab[base + 4]);
					int vh = GetWord(&tab[base + 6]);
					sensor->valueHd = vl | (vh << 16);
					sensor->valueFiz = sensor->valueHd * getMeasFactor(sensor->measType);
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
}

void GasS873::doOnTimeOut() {
	if (autoRd.phase > 0) {
		autoRd.phase = 0;
		shellTask->oMsgX(colRED, "MDB%u: read measure TIMEOUT", mMdbNr);
		strcpy(autoRd.statusTxt, "TimeOut");
	} else {
		shellTask->oMsgX(colRED, "MDB%u: TIMEOUT", mMdbNr);
	}
}

void GasS873::loopFunc() {
	if (isAnyConfiguredData()) {
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
					sendMdbFun4(reqSYS, config->data.R.rest.gasDevMdbNr, 1, 4); //todo było 1,5
					break;
				case 3:
					sendMdbFun4(reqSYS, config->data.R.rest.gasDevMdbNr, 1001, 4 * gasData.devCntTh);
					autoRd.phase = 4;
					break;
				case 4:
				case 2:
					if (HAL_GetTick() - state.sent.tick > MAX_TIME_REPL)
						autoRd.phase = 0;
					break;
				}
			}
		}
	}
}

void GasS873::showState(OutStream *strm) {
	MdbMasterTask::showState(strm);
	if (strm->oOpen(colWHITE)) {
		strm->oMsg("RedCnt=%d", autoRd.redCnt);
		strm->oMsg("TmRd=%.2f[s]", (HAL_GetTick() - autoRd.redTick) / 1000.0);
		strm->oClose();

	}
}

bool GasS873::isMeasValid(uint16_t status) {
	return ((status & 0x07) == 0);
}

const char* GasS873::getSensValidStr(uint16_t status) {
	if ((status & 0x07) != 0)
		return "ERR";
	else if ((status & 0x3FF) != 0)
		return "WARN";
	else
		return "OK";
}

void GasS873::showMeas(OutStream *strm) {
	if (strm->oOpen(colWHITE)) {
		strm->oMsg("RedCnt=%d", autoRd.redCnt);
		strm->oMsg("TmRd=%.2f[s]", (HAL_GetTick() - autoRd.redTick) / 1000.0);
		strm->oMsg("MeasCnt=%d (%d)", gasData.devCnt, gasData.devCntTh);
		strm->oMsg("serialNum=%d", gasData.serialNum);
		strm->oMsg("ProdYear=%d", gasData.ProdYear);
		strm->oMsg("FirmwareVer=%d.%03d", gasData.FirmwareVer >> 8, gasData.FirmwareVer & 0xff);
		strm->oMsg("FailureCode=%d", gasData.FailureCode);

		for (int i = 0; i < gasData.devCntTh; i++) {
			S873Data *pD = &gasData.sensorTab[i];
			MeasType mt = pD->measType;

			int n = snprintf(gtxt, sizeof(gtxt), "%u. %12s %3s Status=0x%04X ", i + 1, //
			GlobData::GetMeasName(mt), getSensValidStr(pD->Status), pD->Status);

			if (isMeasValid(pD->Status)) {
				n += snprintf(&gtxt[n], sizeof(gtxt) - n, "v=%5d  ", pD->valueHd);
				n += snprintf(&gtxt[n], sizeof(gtxt) - n, GlobData::GetMeasPrecisionStr(mt), pD->valueFiz);
				n += snprintf(&gtxt[n], sizeof(gtxt) - n, "[%s]", GlobData::GetMeasUnit(mt));

				if (pD->filtrFIR != NULL) {
					n += snprintf(&gtxt[n], sizeof(gtxt) - n, " (FIR:%.3f) (IR:%.3f)", pD->filtrFIR->out(), pD->filtrIR->out());
				}
			}
			strm->oMsg(gtxt);

		}

		strm->oClose();
	}
}

bool GasS873::getMeasValue(MeasType measType, float *val) {
	return getMeasValue(measType, config->data.R.exDev.gasFiltrType, val);
}

bool GasS873::getMeasValue(MeasType measType, int filtrType, float *val) {

	if (isDataError()) {
		*val = NAN;
		return false;
	}

	for (int i = 0; i < gasData.devCntTh; i++) {
		if (measType == gasData.sensorTab[i].measType) {
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

const ShellItem menuGas[] = { //
		{ "m", "pomiary" }, //

				{ NULL, NULL } };

const ShellItem* GasS873::getMenu() {
	if (menu.tab == NULL) {
		buildMenu(menuGas);
	}
	return menu.tab;
}

bool GasS873::execMyMenuItem(OutStream *strm, int idx, const char *cmd) {
	switch (idx) {
	case 0:  //m
		showMeas(strm);
		break;
	default:
		return false;
	}
	return true;
}

bool GasS873::execMenuItem(OutStream *strm, int idx, const char *cmd) {
	bool q = MdbMasterTask::execMenuItem(strm, idx, cmd);
	if (!q) {
		q = execMyMenuItem(strm, idx - menu.baseCnt, cmd);
	}
	return q;
}

const char* GasS873::getMenuName() {
	return "GasS873 sensor Menu";
}

void GasS873::getDeviceStatusTxt(char *txt, int max) {
	snprintf(txt, max, "ReqCnt=%u RdCnt=%u TimeOutCnt=%u ST=%s", autoRd.reqCnt, autoRd.redCnt, state.timeOutCnt, autoRd.statusTxt);
}
