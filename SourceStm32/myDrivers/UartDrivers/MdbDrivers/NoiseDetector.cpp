/*
 * NoiseDetector.cpp
 *
 *  Created on: 8 gru 2022
 *      Author: Grzegorz
 */

#include <NoiseDetector.h>

#if (DEV_NOISE)

#include "utils.h"
#include "main.h"
#include "UMain.h"
#include "Config.h"

#include "string.h"
#include "stdio.h"
#include "math.h"

extern Config *config;

//-----------------------------------------------------------------------------------------
// MdbMasterNoiseTask
//-----------------------------------------------------------------------------------------
NoiseDetector::NoiseDetector(MdbMasterTask *mdbTask, uint8_t mdbAdr, const char *name) :
		MdbDev::MdbDev(mdbTask, mdbAdr, name) {
	memset(&autoRd, 0, sizeof(autoRd));
	memset(&noiseData, 0, sizeof(noiseData));

	strcpy(autoRd.statusTxt, "Start");
}


void NoiseDetector::onReciveData(bool replOK, uint8_t mdbFun, const uint8_t *tab, int regCnt) {
	if (autoRd.phase > 0) {
		if (replOK) {

			switch (autoRd.phase) {
			case 2: {
				noiseData.valueHd = GetWord(&tab[0]);
				noiseData.valueFiz = noiseData.valueHd / 10.0;

				if (noiseData.filtrIR == NULL)
					noiseData.filtrIR = new FiltrIR(config->data.R.rest.noiseFiltrIRConst);
				if (noiseData.filtrFIR == NULL)
					noiseData.filtrFIR = new FiltrFIR(config->data.R.rest.noiseFiltrFIRLength);
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

void NoiseDetector::onTimeOut() {
	if (autoRd.phase > 0) {
		autoRd.phase = 0;
		getOutStream()->oMsgX(colRED, "MDB%u: read measure TIMEOUT", getMdbNr());
		strcpy(autoRd.statusTxt, "TimeOut");
	} else {
		getOutStream()->oMsgX(colRED, "MDB%u: TIMEOUT", getMdbNr());
	}
}

void NoiseDetector::loopFunc() {
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
					sendMdbFun4(reqSYS, 1, 1, 1); // jeden rejestr od początku pamięci
					break;
				case 2:
					if (HAL_GetTick() - getSentTick() > MAX_TIME_REPL)
						autoRd.phase = 0;
					break;
				}
			}
		}
	}
}

void NoiseDetector::showState(OutStream *strm) {
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

		if (!isDataError()) {
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

bool NoiseDetector::isDataError() {
	return ((autoRd.redTick == 0) || (HAL_GetTick() - autoRd.redTick > TIME_MEAS_VALID));
}

bool NoiseDetector::isMeasServiced(MeasType measType){
	return (measType == ssNOISE);
}



bool NoiseDetector::getMeasValue(MeasType measType, float *val) {
	if (measType != ssNOISE) {
		return false;
	}

	if (isDataError()) {
		return false;
	}
	int filtrType = config->data.R.rest.noiseFiltrType;

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

void NoiseDetector::funShowMeasure(OutStream *strm, const char *cmd, void *arg) {
	NoiseDetector *dev = (NoiseDetector*) arg;
	dev->showMeas(strm);
}

const ShellItemFx menuNoiseFx[] = { //
		{ "m", "pomiary", NoiseDetector::funShowMeasure }, //
				{ NULL, NULL } };

void NoiseDetector::shell(OutStream *strm, const char *cmd) {
	execMenuCmd(strm, menuNoiseFx, cmd, this, "Noise sensor Menu");
}

#endif
