/*
 * NoiseDetector.cpp
 *
 *  Created on: 8 gru 2022
 *      Author: Grzegorz
 */

#include <NoiseDetector.h>

#include "utils.h"
#include "main.h"
#include "UMain.h"
#include "shell.h"
#include "Config.h"
//#include "i2cDev.h"
//#include "Hal.h"

#include "string.h"
#include "stdio.h"
#include "math.h"

extern ShellTask *shellTask;
extern Config *config;

//-----------------------------------------------------------------------------------------
// MdbMasterNoiseTask
//-----------------------------------------------------------------------------------------
NoiseDetector::NoiseDetector(int mdbNr, int portNr) :
		MdbMasterTask::MdbMasterTask(mdbNr, portNr) {
	memset(&autoRd, 0, sizeof(autoRd));
	memset(&noiseData, 0, sizeof(noiseData));

	strcpy(autoRd.statusTxt, "Start");
	menu.tab = NULL;
	menu.baseCnt = 0;
}

bool NoiseDetector::isCfgNoiseOn() {
	return (config->data.R.exDev.sensExist[ssNOISE]);
}

void NoiseDetector::onReciveData(bool replOK, uint8_t mdbFun, const uint8_t *tab, int regCnt) {
	if (autoRd.phase > 0) {
		if (replOK) {

			switch (autoRd.phase) {
			case 2: {
				noiseData.valueHd = GetWord(&tab[0]);
				noiseData.valueFiz = noiseData.valueHd / 10.0;

				if (noiseData.filtrIR == NULL)
					noiseData.filtrIR = new FiltrIR(config->data.R.exDev.noiseFiltrIRConst);
				if (noiseData.filtrFIR == NULL)
					noiseData.filtrFIR = new FiltrFIR(config->data.R.exDev.noiseFiltrFIRLength);
				noiseData.filtrIR->inp(noiseData.valueFiz);
				noiseData.filtrFIR->inp(noiseData.valueFiz);

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

void NoiseDetector::doOnTimeOut() {
	if (autoRd.phase > 0) {
		autoRd.phase = 0;
		shellTask->oMsgX(colRED, "MDB%u: read measure TIMEOUT", mMdbNr);
		strcpy(autoRd.statusTxt, "TimeOut");
	} else {
		shellTask->oMsgX(colRED, "MDB%u: TIMEOUT", mMdbNr);
	}
}

void NoiseDetector::loopFunc() {
	if (isCfgNoiseOn()) {
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
					sendMdbFun4(reqSYS, 1, 1, 1); // jeden rejestr od początku pamięci
					break;
				case 2:
					if (HAL_GetTick() - state.sent.tick > MAX_TIME_REPL)
						autoRd.phase = 0;
					break;
				}
			}
		}
	}
}

void NoiseDetector::showState(OutStream *strm) {
	MdbMasterTask::showState(strm);
	if (strm->oOpen(colWHITE)) {
		strm->oMsg("RedCnt=%d", autoRd.redCnt);
		strm->oMsg("TmRd=%.2f[s]", (HAL_GetTick() - autoRd.redTick) / 1000.0);
		strm->oClose();

	}
}

void NoiseDetector::showMeas(OutStream *strm) {
	if (strm->oOpen(colWHITE)) {
		strm->oMsg("ReqCnt=%d", autoRd.reqCnt);
		strm->oMsg("RedCnt=%d", autoRd.redCnt);
		strm->oMsg("TmRd=%.2f[s]", (HAL_GetTick() - autoRd.redTick) / 1000.0);

		int n = snprintf(gtxt, sizeof(gtxt), "Noise: ");

		if (!isError()) {
			n += snprintf(&gtxt[n], sizeof(gtxt) - n, "%.1f[dB]", noiseData.valueFiz);
			if (noiseData.filtrFIR != NULL) {
				n += snprintf(&gtxt[n], sizeof(gtxt) - n, " (FIR:%.1f) (IR:%.1f)", noiseData.filtrFIR->out(), noiseData.filtrIR->out());
			}
		} else {
			n += snprintf(&gtxt[n], sizeof(gtxt) - n, "brak danych");
		}
		strm->oMsg(gtxt);
	}

	strm->oClose();

}

bool NoiseDetector::isError() {
	return ((autoRd.redTick == 0) || (HAL_GetTick() - autoRd.redTick > TIME_MEAS_VALID));
}

bool NoiseDetector::getNoiseValue(float *val) {
	return getNoiseValue(config->data.R.exDev.noiseFiltrType, val);
}

bool NoiseDetector::getNoiseValue(int filtrType, float *val) {

	if (isError()) {
		*val = NAN;
		return false;
	}

	float v = noiseData.valueFiz;
	switch (filtrType) {
	case 1:
		if (noiseData.filtrFIR != NULL)
			v = noiseData.filtrFIR->out();
		break;
	case 2:
		if (noiseData.filtrIR != NULL)
			v = noiseData.filtrIR->out();
		break;
	}
	*val = v;
	return true;

}

const ShellItem menuNoise[] = { //
		{ "m", "pomiary" }, //

				{ NULL, NULL } };

const ShellItem* NoiseDetector::getMenu() {
	if (menu.tab == NULL) {
		buildMenu(menuNoise);
	}
	return menu.tab;
}

bool NoiseDetector::execMyMenuItem(OutStream *strm, int idx, const char *cmd) {
	switch (idx) {
	case 0:  //m
		showMeas(strm);
		break;
	default:
		return false;
	}
	return true;
}

bool NoiseDetector::execMenuItem(OutStream *strm, int idx, const char *cmd) {
	bool q = MdbMasterTask::execMenuItem(strm, idx, cmd);
	if (!q) {
		q = execMyMenuItem(strm, idx - menu.baseCnt, cmd);
	}
	return q;
}

const char* NoiseDetector::getMenuName() {
	return "Noise sensor Menu";
}

