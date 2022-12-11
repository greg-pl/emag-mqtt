/*
 * DustPMSA.cpp
 *
 *  Created on: Dec 7, 2020
 *      Author: Grzegorz
 */
#include <string.h>
#include <math.h>

#include "DustPMSA.h"
#include "utils.h"
#include "shell.h"
#include "ShellItem.h"
#include "Hal.h"

extern ShellTask *shellTask;

//kody błędów zwracane przez PMSA
enum {
	pmsa_OK = 0, //
	pmsa_Wrong_Data_Len = 1,  //
	pmsa_Unkn_Cmd = 2, //
	pmsa_No_access = 3, //
	pmsa_Bad_param = 4, //
	pmsa_Internal_Func = 0x28, //
	pmsa_Command_Not_Allowed = 0x43, //
};

DustPMSA::DustPMSA(bool formal_exist) :
		DustSensorBase::DustSensorBase(), TUart::TUart(TUart::myUART5, 7) {

	mFormaldehydeExist = formal_exist;
	memset(&state, 0, sizeof(state));
	memset(&rxRec, 0, sizeof(rxRec));
	memset(&txRec, 0, sizeof(txRec));
	mShowMeasCnt = 0;

}

void DustPMSA::startRecive() {
	HAL_UART_Receive_IT(&mHuart, &rxRec.mRecByte, 1);
}

HAL_StatusTypeDef DustPMSA::Init(SignaledClass *signObj) {
	mSignObj = signObj;
	HAL_StatusTypeDef st = TUart::Init(9600);
	if (st == HAL_OK) {
		startRecive();
	}
	Hdw::dustSleepOn(false);
	return st;
}

void DustPMSA::ErrorCallback() {

}

void DustPMSA::RxCpltCallback() {
	state.rxCnt++;
	if (rxRec.rxPtr < (int) sizeof(rxRec.buf)) {
		rxRec.buf[rxRec.rxPtr++] = rxRec.mRecByte;
		rxRec.rxTick = HAL_GetTick();
	}
	startRecive();
	mSignObj->setSignal();
}

void DustPMSA::TxCpltCallback() {
	TUart::TxCpltCallback();
	state.txCmplCnt++;
}

void DustPMSA::SendCmd(uint8_t cmd, uint16_t arg) {
	if (!isSending()) {

		txRec.buf[0] = 0x42;
		txRec.buf[1] = 0x4d;
		txRec.buf[2] = cmd;
		txRec.buf[3] = arg >> 8;
		txRec.buf[4] = arg & 0xff;

		int sum = 0;
		int len = 5;
		for (int i = 0; i < len; i++) {
			sum += txRec.buf[i];
		}
		txRec.buf[len++] = sum >> 8;  // dołożenie sumy kontrolnej
		txRec.buf[len++] = sum & 0xff;

		writeBuf((uint8_t*) &txRec.buf, len);
	}
}

void DustPMSA::sendGetMeasure() {
	if (!isSending()) {
		state.measFrame.getMeasSended = true;
		state.measFrame.sendGetReqMeasTick = HAL_GetTick();
		SendCmd(0xE2, 1);
	}
}

void DustPMSA::sendGoSleep() {
	SendCmd(0xE4, 0);
}
void DustPMSA::sendWakeUp() {
	SendCmd(0xE4, 1);
}

void DustPMSA::sendSetActiveMode() {
	if (!isSending()) {
		SendCmd(0xE1, 1);
		state.measStart.startMeasureTick = HAL_GetTick();
		state.measStart.startMeasureSended = true;
	}
}

void DustPMSA::sendSetPassiveMode() {
	if (!isSending()) {
		SendCmd(0xE1, 0);
		state.measStart.startMeasureSended = false;
	}
}

static uint16_t getWord(uint8_t *ptr) {
	return (ptr[0] << 8) | (ptr[1]);
}

void DustPMSA::StartMeas() {
	state.isMeasureOn = true;
	state.measureOnTick = HAL_GetTick();
	state.measStart.startMeasureSended = false;
}
void DustPMSA::StopMeas() {
	state.isMeasureOn = false;
	state.measStart.startMeasureSended = false;
	state.measureOffTick = HAL_GetTick();
}

void DustPMSA::setPower(bool on) {
	state.isPowerOn = on;
	state.powerChgTick = HAL_GetTick();
	Hdw::dustSensorOn(on);
}

void DustPMSA::ShowMesuredRec() {
	if (shellTask->oOpen(colWHITE)) {
		shellTask->oMsg("--- N=%u ------TT=%u--(dt=%u)---------", mShowMeasCnt, (int) HAL_GetTick(), state.measFrame.getMeasTick - state.measFrame.prevMeasTick);

		shellTask->oMsg("PM1.0 : %.0f[ug/m3]", measHpma.PM_1_0);
		shellTask->oMsg("PM2.5 : %.0f[ug/m3]", measHpma.PM_2_5);
		shellTask->oMsg("PM10  : %.0f[ug/m3]", measHpma.PM_10);
		shellTask->oMsg("Atmosferic environment");
		shellTask->oMsg("PM1.0 : %.0f[ug/m3]", measHpma.PM_AE_1_0);
		shellTask->oMsg("PM2.5 : %.0f[ug/m3]", measHpma.PM_AE_2_5);
		shellTask->oMsg("PM10  : %.0f[ug/m3]", measHpma.PM_AE_10);
		shellTask->oMsg("Number of particles");
		shellTask->oMsg("Num D<0.3mm  : %u", measHpma.NUM_D03);
		shellTask->oMsg("Num D<0.5mm  : %u", measHpma.NUM_D05);
		shellTask->oMsg("Num D<1.0mm  : %u", measHpma.NUM_D10);
		shellTask->oMsg("Num D<2.5mm  : %u", measHpma.NUM_D25);
		shellTask->oMsg("Num D<5.0mm  : %u", measHpma.NUM_D50);
		shellTask->oMsg("Num D<10.0mm : %u", measHpma.NUM_D100);
		if (!mFormaldehydeExist) {
			shellTask->oMsg("reserved     : %u", measHpma.Reserved);
		} else {
			shellTask->oMsg("Formaldehyde : %.3f[mg/m3]", measHpma.Formaldehyde);
			shellTask->oMsg("Temperature  : %.1f[st.C]", measHpma.Temper);
			shellTask->oMsg("Humidity     : %.1f[%%]", measHpma.Humidity);
			shellTask->oMsg("Firm.Ver     : %u", measHpma.FirmwareVer);
			shellTask->oMsg("ErrorCode    : %u", measHpma.ErrorCode);
		}
		shellTask->oClose();
	}

}

HAL_StatusTypeDef DustPMSA::execNewFrame() {

//sprawdzenie sumy kontrolnej
	state.recFrameCnt++;
	int cnt = rxRec.rxPtr;

	if ((rxRec.buf[0] == 0x42) || (rxRec.buf[1] == 0x4d)) {
		uint16_t sum = 0;
		for (int i = 0; i < cnt - 2; i++) {
			sum += rxRec.buf[i];
		}
		uint16_t sum2 = getWord(&rxRec.buf[cnt - 2]);

		if (sum == sum2) {
			state.recFrameSumOkCnt++;

			bool frameOk;
			if (!mFormaldehydeExist)
				frameOk = (cnt == 32);
			else
				frameOk = (cnt == 40);

			if (frameOk) {
				//uint16_t frameLen = getWord(&rxRec.buf[2]);
				bool canShow = false;

				if (openMutex(50)) {
					uint32_t tt = HAL_GetTick();

					measHpma.PM_1_0 = getWord(&rxRec.buf[4]);
					measHpma.PM_2_5 = getWord(&rxRec.buf[6]);
					measHpma.PM_10 = getWord(&rxRec.buf[8]);

					measHpma.PM_AE_1_0 = getWord(&rxRec.buf[10]);
					measHpma.PM_AE_2_5 = getWord(&rxRec.buf[12]);
					measHpma.PM_AE_10 = getWord(&rxRec.buf[14]);

					measHpma.NUM_D03 = getWord(&rxRec.buf[16]);
					measHpma.NUM_D05 = getWord(&rxRec.buf[18]);
					measHpma.NUM_D10 = getWord(&rxRec.buf[20]);
					measHpma.NUM_D25 = getWord(&rxRec.buf[22]);
					measHpma.NUM_D50 = getWord(&rxRec.buf[24]);
					measHpma.NUM_D100 = getWord(&rxRec.buf[26]);

					if (!mFormaldehydeExist) {
						measHpma.Reserved = getWord(&rxRec.buf[28]);
					} else {
						measHpma.Formaldehyde = 0.001 * getWord(&rxRec.buf[28]);
						measHpma.Temper = 0.1 * getWord(&rxRec.buf[30]);
						measHpma.Humidity = 0.1 * getWord(&rxRec.buf[32]);
						measHpma.Reserved = getWord(&rxRec.buf[34]);
						measHpma.FirmwareVer = rxRec.buf[36];
						measHpma.ErrorCode = rxRec.buf[37];
					}

					exportDt.filterPM1_0.inp(measHpma.PM_1_0);
					exportDt.filterPM2_5.inp(measHpma.PM_2_5);
					exportDt.filterPM10.inp(measHpma.PM_10);
					exportDt.mLastRdDataTick = tt;
					if (mFormaldehydeExist) {
						exportDt.filterFormaldehyde.inp(measHpma.Formaldehyde);
					} else {
						exportDt.filterFormaldehyde.inp(0);
					}

					state.measFrame.measRecivedTick = tt;
					state.measFrame.prevMeasTick = state.measFrame.getMeasTick;
					state.measFrame.getMeasTick = tt;
					state.measFrame.getMeasSended = false;

					canShow = true;
					closeMutex();
				}
				if (canShow) {
					if (mShowMeasCnt > 0) {
						mShowMeasCnt--;
						ShowMesuredRec();
					}
				}
			} else {
				state.recFrameLenErr++;

			}

			return HAL_OK;

		} else
			return HAL_SUM_ERR;
	} else
		return HAL_DATA_ERR;
}

void DustPMSA::tick() {
	state.loopCnt++;
	uint32_t tt = HAL_GetTick();

	if (rxRec.rxPtr > 0 && tt - rxRec.rxTick > 10) {
		execNewFrame();
		rxRec.rxPtr = 0;
	}

	if (state.isPowerOn) {

		if (state.isMeasureOn) {
			if (tt - state.measureOnTick > 500) {
				if (!state.measStart.startMeasureSended) {
					sendSetActiveMode();
				} else {
					if (tt - state.measStart.startMeasureTick > 500) {

						if ((state.measFrame.sendGetReqMeasTick == 0) || (tt - state.measFrame.sendGetReqMeasTick > 2000)) {
							sendGetMeasure();
						}
						if ((tt - exportDt.mLastRdDataTick > MAX_MEAS_VALID) && (tt - state.powerChgTick > 10000)) {
							//jesli brak danych to wyłączenie zasilania
							setPower(false);
							uartAbort();
							state.powerOffCnt++;
						}
					}
				}
			}
		} else {
			if (tt - state.measureOffTick > 1000) {
				StartMeas();
			}
		}
	} else {
		//włączenie zasilania
		if (tt - state.powerChgTick > 500) {
			StopMeas();
			setPower(true);
			startRecive();
		}
	}

}

void DustPMSA::showState(OutStream *strm) {
	if (strm->oOpen(colWHITE)) {
		strm->oMsg("___PMSA Status___");
		strm->oMsg("DUST_ON:%s", YN(Hdw::getDustSensorOn()));
		strm->oMsg("DUST_FLG:%s", ErrOk(Hdw::getDustSensorFlg()));
		strm->oMsg("DUST_SLEEP#:%s", YN(Hdw::getDustSleepOn()));

		strm->oMsg("loopCnt=%u", state.loopCnt);
		strm->oMsg("IrqCnt=%u", mIrqCnt);
		strm->oMsg("txCmplCnt=%u", state.txCmplCnt);
		strm->oMsg("rxCnt=%u", state.rxCnt);
		strm->oMsg("recFrameCnt=%u", state.recFrameCnt);
		strm->oMsg("recFrameSumOkCnt=%u", state.recFrameSumOkCnt);
		strm->oMsg("recFrameLenErr=%u", state.recFrameLenErr);
		strm->oMsg("powerOffCnt=%u", state.powerOffCnt);

		strm->oClose();
	}
}

void DustPMSA::clrState() {

	state.loopCnt = 0;
	mIrqCnt = 0;
	state.txCmplCnt = 0;
	state.rxCnt = 0;
	state.recFrameCnt = 0;
	state.recFrameSumOkCnt = 0;
	state.recFrameLenErr = 0;
	state.powerOffCnt = 0;
}

void DustPMSA::funShowState(OutStream *strm, const char *cmd, void *arg) {
	DustPMSA *dev = (DustPMSA*) arg;
	dev->showState(strm);
}
void DustPMSA::funSetPower(OutStream *strm, const char *cmd, void *arg) {
	int a;
	Token::getAsInt(&cmd, &a);
	Hdw::dustSensorOn(a);
	strm->oMsg("DustPower=%u", a);
}
void DustPMSA::funGetMeas(OutStream *strm, const char *cmd, void *arg) {
	DustPMSA *dev = (DustPMSA*) arg;
	dev->sendGetMeasure();
}
void DustPMSA::funWakeup(OutStream *strm, const char *cmd, void *arg) {
	DustPMSA *dev = (DustPMSA*) arg;
	dev->sendWakeUp();
}

void DustPMSA::funGoSleep(OutStream *strm, const char *cmd, void *arg) {
	DustPMSA *dev = (DustPMSA*) arg;
	dev->sendGoSleep();
}
void DustPMSA::funSetActiveMode(OutStream *strm, const char *cmd, void *arg) {
	DustPMSA *dev = (DustPMSA*) arg;
	dev->sendSetActiveMode();
}
void DustPMSA::funSetPassiveMode(OutStream *strm, const char *cmd, void *arg) {
	DustPMSA *dev = (DustPMSA*) arg;
	dev->sendSetPassiveMode();
}
void DustPMSA::funSetHdSleep(OutStream *strm, const char *cmd, void *arg) {
	int a;
	Token::getAsInt(&cmd, &a);
	Hdw::dustSleepOn(a);
	strm->oMsg("DustHDSleep=%u", a);
}
void DustPMSA::funShowNmMeas(OutStream *strm, const char *cmd, void *arg) {
	DustPMSA *dev = (DustPMSA*) arg;
	dev->mShowMeasCnt = 1;
	Token::getAsInt(&cmd, &dev->mShowMeasCnt);
	strm->oMsg("ShowMeasCnt=%u", dev->mShowMeasCnt);
}
void DustPMSA::funClrStat(OutStream *strm, const char *cmd, void *arg) {
	DustPMSA *dev = (DustPMSA*) arg;
	dev->showState(strm);
}

const ShellItemFx menuDustFx[] = { //
		{ "s", "stan", DustPMSA::funShowState }, //
				{ "pwr", "ustaw zasilanie czujnika", DustPMSA::funSetPower }, //
				{ "get", "send get measure", DustPMSA::funGetMeas }, //
				{ "wakeup", "wakeup sensor", DustPMSA::funWakeup }, //
				{ "sleep", "sleep sensor", DustPMSA::funGoSleep }, //
				{ "active", "set active mode", DustPMSA::funSetActiveMode }, //
				{ "passive", "set passive mode", DustPMSA::funSetPassiveMode }, //
				{ "hd_sleep", "set hardware sleep line", DustPMSA::funSetHdSleep }, //
				{ "meas", "pokaz n pomiarów", DustPMSA::funShowNmMeas }, //
				{ "clrCnt", "zeruj liczniki statystyki", DustPMSA::funClrStat }, //

				{ NULL, NULL } };

void DustPMSA::shell(OutStream *strm, const char *cmd) {
	execMenuCmd(strm, menuDustFx, cmd, this, "Dust sensor PMSA Menu");
}

