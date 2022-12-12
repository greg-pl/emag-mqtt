/*
 * SPS30.cpp
 *
 *  Created on: 17 mar 2021
 *      Author: Grzegorz
 */

#include "SPS30.h"
#include "utils.h"
#include "ShellItem.h"
#include "Hal.h"
#include "Token.h"

#include <string.h>


//kody błędów zwracane przez SPS30
enum {
	sps30_OK = 0, //
	sps30_Wrong_Data_Len = 1,  //
	sps30_Unkn_Cmd = 2, //
	sps30_No_access = 3, //
	sps30_Bad_param = 4, //
	sps30_Internal_Func = 0x28, //
	sps30_Command_Not_Allowed = 0x43, //
};

SPS30::SPS30() :
		DustSensorBase::DustSensorBase("SPS30"), TUart::TUart(TUart::myUART5, 7) {
	memset(&state, 0, sizeof(state));
	memset(&rxRec, 0, sizeof(rxRec));
	memset(&txRec, 0, sizeof(txRec));
	memset(&measSps30, 0, sizeof(measSps30));

	mShowMeasCnt = 0;
	mDebug = 0;
}

HAL_StatusTypeDef SPS30::Init(SignaledClass *signObj) {
	mSignObj = signObj;
	HAL_StatusTypeDef st = TUart::Init(115200);
	if (st == HAL_OK) {
		HAL_UART_Receive_IT(&mHuart, &rxRec.mRecByte, 1);
	}
	Hdw::dustSleepOn(true);
	Hdw::dustSensorOn(false);
	osDelay(100);
	Hdw::dustSensorOn(true);

	return st;
}

void SPS30::ErrorCallback() {

}

void SPS30::TxCpltCallback() {
	state.txCmplCnt++;
	txRec.mSending = false;
}

void SPS30::RxCpltCallback() {
	state.rxCnt++;
	if (!rxRec.frameComplete) {
		if (rxRec.rxPtr == 0) {
			if (rxRec.mRecByte == START_BYTE) {
				rxRec.buf[rxRec.rxPtr++] = rxRec.mRecByte;
			}
		} else {
			if (rxRec.rxPtr < (int) sizeof(rxRec.buf)) {
				rxRec.buf[rxRec.rxPtr] = rxRec.mRecByte;
				rxRec.rxPtr++;
				if (rxRec.mRecByte == START_BYTE)
					rxRec.frameComplete = true;
			}
		}
	}
	HAL_UART_Receive_IT(&mHuart, &rxRec.mRecByte, 1);
}

void SPS30::putTxFrame(uint8_t a) {
	switch (a) {
	case 0x7E:
		txRec.buf[txRec.inPtr++] = 0x7D;
		txRec.buf[txRec.inPtr++] = 0x5E;
		break;
	case 0x7D:
		txRec.buf[txRec.inPtr++] = 0x7D;
		txRec.buf[txRec.inPtr++] = 0x5D;
		break;
	case 0x11:
		txRec.buf[txRec.inPtr++] = 0x7D;
		txRec.buf[txRec.inPtr++] = 0x31;
		break;
	case 0x13:
		txRec.buf[txRec.inPtr++] = 0x7D;
		txRec.buf[txRec.inPtr++] = 0x33;
		break;
	default:
		txRec.buf[txRec.inPtr++] = a;
		break;
	}
}

void SPS30::SendFrame(uint8_t cmd, const uint8_t *dt, uint8_t len) {
	txRec.inPtr = 0;
	txRec.buf[txRec.inPtr++] = START_BYTE;
	putTxFrame(SLAVE_ADR);
	putTxFrame(cmd);
	putTxFrame(len);
	if (dt != NULL && len > 0) {
		for (int i = 0; i < len; i++) {
			putTxFrame(dt[i]);
		}
	}
	uint8_t sum = SLAVE_ADR + cmd + len;

	if (dt != NULL && len > 0) {
		for (int i = 0; i < len; i++) {
			sum += dt[i];
		}
	}
	sum = sum ^ 0xFF;
	putTxFrame(sum);
	txRec.buf[txRec.inPtr++] = START_BYTE;
	txRec.mSending = true;
	HAL_UART_Transmit_IT(&mHuart, (uint8_t*) &txRec.buf, txRec.inPtr);
}

uint32_t swap(uint32_t v) {
	uint32_t x;
	uint8_t *src = (uint8_t*) &v;
	uint8_t *dst = (uint8_t*) &x;
	dst[0] = src[3];
	dst[1] = src[2];
	dst[2] = src[1];
	dst[3] = src[0];
	return x;
}

void SPS30::setPower(bool on) {
	Hdw::dustSensorOn(on);
}

TStatus SPS30::execNewFrame() {
	char frame[80];

	if (rxRec.buf[0] != START_BYTE || rxRec.buf[rxRec.rxPtr - 1] != START_BYTE)
		return stERR_NO_START_BYTE;

	//pominięcie warstwy SHDLC
	int ptr = 0;
	int inp = 1;
	while (inp < rxRec.rxPtr - 1) {
		uint8_t a = rxRec.buf[inp++];
		if (a == 0x7D) {
			uint8_t a1 = rxRec.buf[inp++];
			switch (a1) {
			case 0x5E:
				frame[ptr++] = 0x7E;
				break;
			case 0x5D:
				frame[ptr++] = 0x7D;
				break;
			case 0x31:
				frame[ptr++] = 0x11;
				break;
			case 0x33:
				frame[ptr++] = 0x13;
				break;
			}

		} else
			frame[ptr++] = a;
		if (ptr >= (int) sizeof(frame))
			return stFRAME_TOO_BIG;
	}

	//sprawdzenie sumy kontrolnej
	uint8_t sum = 0;
	for (int i = 0; i < ptr - 1; i++) {
		sum += frame[i];
	}
	sum = sum ^ 0xFF;
	if (sum != frame[ptr - 1])
		return stERR_SUM;

	state.recivedFrameCnt++;

	//rozbiór logiczny danych
	uint8_t cmd = frame[1];
	uint8_t dtLen = ptr - 5;
	uint8_t dtLen1 = frame[3];
	char *frDt = &frame[4];

	state.devState = frame[2];
	if (mDebug >= 2)
		getOutStream()->oMsgX(colMAGENTA, "rec cmd=0x%02X state=0x%02X L=%d(%d)", cmd, state.devState, dtLen, dtLen1);
	if (mDebug >= 3) {
		char txt[100];
		int n = snprintf(txt, sizeof(txt), "DT:");

		for (int i = 0; i < dtLen; i++) {
			n += snprintf(&txt[n], sizeof(txt) - n, "0x%02X,", frDt[i]);
		}
		getOutStream()->oMsgX(colMAGENTA, txt);
	}

	switch (cmd) {
	case cmdSTART_MEASURE:
		if (mDebug >= 1) {
			getOutStream()->oMsgX(colMAGENTA, "SP30:StartMeasureAck");
		}
		break;
	case cmdSTOP_MEASURE:
		if (mDebug >= 1) {
			getOutStream()->oMsgX(colMAGENTA, "SP30:StopMeasureAck");
		}
		break;
	case cmdREAD_VAL: {
		if (dtLen >= 40) {
			SPS30_Data sps30;
			//wyrównaie położenia zmiennych
			memcpy(&sps30, frDt, sizeof(sps30));
			//Swap dla wszystkich elementów
			for (int i = 0; i < 10; i++) {
				measSps30.tab[i] = swap(sps30.tab[i]);
			}
			state.measFrame.getMeasTick = HAL_GetTick();
			state.measFrame.sendGetReqMeasTick = 0;

			if (mDebug >= 3) {
				ShowMesuredRec(getOutStream());
			}
			if (openMutex(50)) {
				uint32_t tt = HAL_GetTick();

				exportDt.filterPM1_0.inp(measSps30.PM_1_0);
				exportDt.filterPM2_5.inp(measSps30.PM_2_5);
				exportDt.filterPM10.inp(measSps30.PM_10);
				exportDt.mLastRdDataTick = tt;
				closeMutex();
			}
		}
	}
		break;
	case cmdSET_AUTOCLEANING_INTERVAL:
		break;
	case cmdSTART_FAN_CLEANING:
		if (mDebug >= 1) {
			getOutStream()->oMsgX(colMAGENTA, "CleaningAck");
		}
		break;
	case cmdDEVICE_INFO:
		if (mDebug >= 1) {
			getOutStream()->oMsgX(colMAGENTA, "DevInfo=[%s]", frDt);
		}
		break;
	case cmdRESET:
		break;

	}
	return stOK;

}

void SPS30::sendGetDevInfo(uint8_t par) {
	uint8_t dt[1];
	dt[0] = par;

	SendFrame(cmdDEVICE_INFO, dt, 1);
}

void SPS30::sendDoReset() {
	if (!txRec.mSending) {
		SendFrame(cmdRESET, NULL, 0);
		state.measStart.startMeasureSended = false;
	}
}

void SPS30::sendStartMeasure() {
	if (!txRec.mSending) {
		const uint8_t dt[] = { 1, 3 };

		SendFrame(cmdSTART_MEASURE, dt, 2);
		state.measStart.startMeasureTick = HAL_GetTick();
		state.measStart.startMeasureSended = true;
	}
}

void SPS30::sendStopMeasure() {
	if (!txRec.mSending) {
		SendFrame(cmdSTOP_MEASURE, NULL, 0);
		state.measStart.startMeasureSended = false;
	}
}

void SPS30::sendGetMeasure() {
	if (!txRec.mSending) {
		state.measFrame.sendGetReqMeasTick = HAL_GetTick();
		SendFrame(cmdREAD_VAL, NULL, 0);
	}
}

void SPS30::sendRunCleaning() {
	SendFrame(cmdSTART_FAN_CLEANING, NULL, 0);
}

void SPS30::tick() {
	dword tt = HAL_GetTick();
	if (rxRec.frameComplete) {
		execNewFrame();

		rxRec.frameComplete = false;
		rxRec.rxPtr = 0;
	}

	if (state.isMeasOn) {
		if (tt - state.powerOnTick > 500) {
			if (!state.measStart.startMeasureSended) {
				sendStartMeasure();
			} else {
				if (tt - state.measStart.startMeasureTick > 500) {
					if ((state.measFrame.sendGetReqMeasTick == 0) || (tt - state.measFrame.sendGetReqMeasTick > 1200)) {
						sendGetMeasure();
					}
				}
			}
		}
	}

}

void SPS30::StartMeas() {
	state.isMeasOn = true;
	state.powerOnTick = HAL_GetTick();
	state.measStart.startMeasureSended = false;
}

void SPS30::StopMeas() {
	state.isMeasOn = false;
	state.measStart.startMeasureSended = false;
}

void SPS30::ShowMesuredRec(OutStream *strm) {
	if (strm->oOpen(colWHITE)) {
		strm->oMsg("Mass Concentration");
		strm->oMsg(" PM1.0 : %f[ug/m3]", measSps30.PM_1_0);
		strm->oMsg(" PM2.5 : %f[ug/m3]", measSps30.PM_2_5);
		strm->oMsg(" PM4.0 : %f[ug/m3]", measSps30.PM_4_0);
		strm->oMsg(" PM10  : %f[ug/m3]", measSps30.PM_10);
		strm->oMsg("Number Concentration");
		strm->oMsg(" PM0.5 : %f[#/cm3]", measSps30.NUM_0_5);
		strm->oMsg(" PM1.0 : %f[#/cm3]", measSps30.NUM_1_0);
		strm->oMsg(" PM2.5 : %f[#/cm3]", measSps30.NUM_2_5);
		strm->oMsg(" PM4.0 : %f[#/cm3]", measSps30.NUM_4_0);
		strm->oMsg(" PM10  : %f[#/cm3]", measSps30.NUM_10);
		strm->oMsg("Typical Particle Size :%f[um]", measSps30.ParticleSize);
		strm->oClose();
	}
}

void SPS30::showState(OutStream *strm) {
	if (strm->oOpen(colWHITE)) {
		strm->oMsg("PWR=%u", Hdw::getDustSensorOn());
		strm->oMsg("PWR_FAULT=%u", Hdw::getDustSensorFlg());
		strm->oMsg("rxCnt=%u", state.rxCnt);
		strm->oMsg("txCnt=%u", state.txCmplCnt);
		strm->oMsg("rxPtr=%u", rxRec.rxPtr);
		strm->oMsg("recFrameCnt=%u", state.recivedFrameCnt);
		strm->oMsg("isMeasOn=%u", state.isMeasOn);
		strm->oMsg("PowerTm=%.2f", (HAL_GetTick() - state.powerOnTick) / 1000.0);
		strm->oMsg("devState=0x%02X", state.devState);
		strm->oClose();
	}
}

void SPS30::funShowState(OutStream *strm, const char *cmd, void *arg) {
	SPS30 *dev = (SPS30*) arg;
	dev->showState(strm);
}

void SPS30::funDbgLevel(OutStream *strm, const char *cmd, void *arg) {
	SPS30 *dev = (SPS30*) arg;
	Token::getAsInt(&cmd, &dev->mDebug);
}

void SPS30::funSetPower(OutStream *strm, const char *cmd, void *arg) {
	bool q = false;
	Token::getAsBool(&cmd, &q);
	Hdw::dustSensorOn(q);
}

void SPS30::funGetInfo(OutStream *strm, const char *cmd, void *arg) {
	SPS30 *dev = (SPS30*) arg;
	int nr;
	bool q = false;
	if (Token::getAsInt(&cmd, &nr)) {
		if (nr >= 1 && nr <= 3) {
			dev->sendGetDevInfo(nr);
			q = true;
		}
	}
	if (!q)
		strm->oMsgX(colGREEN, "Polecenie: getinfo 1 [1..3]");
}

void SPS30::funFanClean(OutStream *strm, const char *cmd, void *arg) {
	SPS30 *dev = (SPS30*) arg;
	dev->sendRunCleaning();
}
void SPS30::funRunMeasure(OutStream *strm, const char *cmd, void *arg) {
	SPS30 *dev = (SPS30*) arg;
	Token::getAsBool(&cmd, &dev->state.isMeasOn);

}
void SPS30::funSowMeasure(OutStream *strm, const char *cmd, void *arg) {
	SPS30 *dev = (SPS30*) arg;
	dev->ShowMesuredRec(strm);
}

const ShellItemFx menuDustSPS30Fx[] = { //
		{ "s", "stan", SPS30::funShowState }, //
				{ "dbg", "poziom logów", SPS30::funDbgLevel }, //
				{ "pwr", "0|1 - włącz/wyłącz zasilanie czujnika", SPS30::funSetPower }, //
				{ "getinfo", "1|2|3 -  pobierz informacje z SPS30", SPS30::funGetInfo }, //
				{ "clean", "FAN clean", SPS30::funFanClean }, //
				{ "run", "0|1 - odczytuj pomiary", SPS30::funRunMeasure }, //
				{ "m", "pomiary", SPS30::funSowMeasure }, //

				{ NULL, NULL, NULL } //
		};

void SPS30::shell(OutStream *strm, const char *cmd) {
	execMenuCmd(strm, menuDustSPS30Fx, cmd, this, "Dust sensor SPS30 Menu");
}
