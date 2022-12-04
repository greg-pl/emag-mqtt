/*
 * MdbMaster.cpp
 *
 *  Created on: 6 lut 2021
 *      Author: Grzegorz
 */
#include "string.h"
#include "stdio.h"
#include "math.h"

#include "utils.h"
#include "main.h"
#include "UMain.h"
#include "shell.h"
#include "Config.h"
#include "i2cDev.h"

#include <MdbMasterTask.h>

extern ShellTask *shellTask;
extern Config *config;
extern SHT35DevPub *sht35;
extern Bmp338DevPub *bmp338;

MdbUart::MdbUart(int PortNr, int Priority) :
		TUart::TUart(PortNr, Priority) {
	mThreadId = NULL;
	memset(&rxRec, 0, sizeof(rxRec));
}

HAL_StatusTypeDef MdbUart::Init(int BaudRate, int parity) {
	mUseRts = true;
	mHalfDuplex = false;
	HAL_StatusTypeDef st = TUart::Init(BaudRate, parity);
	rxRec.ptr = 0;

	if (st == HAL_OK) {
		HAL_UART_Receive_IT(&mHuart, (uint8_t*) &rxRec.buf[rxRec.ptr], 1);
	}
	return st;
}

void MdbUart::setTxEn(bool txON) {
	switch (mPortNr) {
	case myUART1:
		if (txON)
			HAL_GPIO_WritePin(USART1_TX_EN_GPIO_Port, USART1_TX_EN_Pin, GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(USART1_TX_EN_GPIO_Port, USART1_TX_EN_Pin, GPIO_PIN_RESET);
		break;
	case myUART3:
		if (txON)
			HAL_GPIO_WritePin(USART3_TX_EN_GPIO_Port, USART3_TX_EN_Pin, GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(USART3_TX_EN_GPIO_Port, USART3_TX_EN_Pin, GPIO_PIN_RESET);
		break;
	case myUART5:
		if (txON)
			HAL_GPIO_WritePin(PMS_SET_GPIO_Port, PMS_SET_Pin, GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(PMS_SET_GPIO_Port, PMS_SET_Pin, GPIO_PIN_RESET);
		break;
	}
}

void MdbUart::TxCpltCallback() {
	TUart::TxCpltCallback();
	setTxEn(false);
}

void MdbUart::RxCpltCallback() {
	rxRec.tick = HAL_GetTick();
	if (rxRec.ptr < FRAME_LEN)
		rxRec.ptr++;
	rxRec.globCnt++;
	HAL_UART_Receive_IT(&mHuart, (uint8_t*) &rxRec.buf[rxRec.ptr], 1);

	if (mThreadId != NULL) {
		osSignalSet(mThreadId, MdbMasterTask::SIGNAL_RXCHAR);
	}

}

void MdbUart::clearRxBuf() {
	rxRec.ptr = 0;
	rxRec.tick = 0;
}

void MdbUart::writeBuf(const void *buf, int len) {
	setTxEn(true);
	TUart::writeBuf(buf, len);
}
//-----------------------------------------------------------------------------------------
// MdbMasterTask
//-----------------------------------------------------------------------------------------

MdbMasterTask::MdbMasterTask(int mdbNr, int portNr) :
		TaskClass::TaskClass("MDB", osPriorityNormal, 1024) {
	char buf[16];
	mMdbNr = mdbNr;
	snprintf(buf, sizeof(buf), "MDB_%u", mdbNr);
	setTaskName(buf);

	mUart = new MdbUart(portNr, 7);
	memset(&state, 0, sizeof(state));
	memset(&reqConsola, 0, sizeof(reqConsola));
	mDbgLevel = 0;
	menu.tab = NULL;
	menu.baseCnt = 0;

}

void MdbMasterTask::Start(int BaudRate, int parity) {
	mUart->Init(BaudRate, parity);
	mDbgLevel = 0;
	TaskClass::Start();
	mUart->setThreadId(getThreadId());
}

void MdbMasterTask::setPower(bool pwr) {
	switch (mMdbNr) {
	case MDB_1:
		if (pwr)
			HAL_GPIO_WritePin(RS1_PWR_EN_GPIO_Port, RS1_PWR_EN_Pin, GPIO_PIN_RESET);
		else
			HAL_GPIO_WritePin(RS1_PWR_EN_GPIO_Port, RS1_PWR_EN_Pin, GPIO_PIN_SET);
		break;
	case MDB_2:
		if (pwr)
			HAL_GPIO_WritePin(RS2_PWR_EN_GPIO_Port, RS2_PWR_EN_Pin, GPIO_PIN_RESET);
		else
			HAL_GPIO_WritePin(RS2_PWR_EN_GPIO_Port, RS2_PWR_EN_Pin, GPIO_PIN_SET);
		break;
	case MDB_3:
		Hdw::dustSensorOn(pwr);
		break;
	}
}

bool MdbMasterTask::getPower() {
	switch (mMdbNr) {
	case MDB_1:
		return (HAL_GPIO_ReadPin(RS1_PWR_EN_GPIO_Port, RS1_PWR_EN_Pin) == GPIO_PIN_RESET);
	case MDB_2:
		return (HAL_GPIO_ReadPin(RS2_PWR_EN_GPIO_Port, RS2_PWR_EN_Pin) == GPIO_PIN_RESET);
	case MDB_3:
		return Hdw::getDustSensorOn();
	}
	return false;
}

bool MdbMasterTask::getPowerFlt() {
	switch (mMdbNr) {
	case MDB_1:
		return (HAL_GPIO_ReadPin(RS1_FLG_GPIO_Port, RS1_FLG_Pin) == GPIO_PIN_RESET);
	case MDB_2:
		return (HAL_GPIO_ReadPin(RS2_FLG_GPIO_Port, RS2_FLG_Pin) == GPIO_PIN_RESET);
	case MDB_3:
		return Hdw::getDustSensorFlg();
	}
	return false;
}

void MdbMasterTask::sendMdbFun(ReqSrc reqSrc, uint8_t *buf, int len) {
	TCrc::Set(buf, len);
	len += 2;
	mUart->clearRxBuf();
	mUart->writeBuf(buf, len);
	state.sent.currReq = reqSrc;
	state.sent.devNr = buf[0];
	state.sent.code = buf[1];
	state.sent.tick = HAL_GetTick();
}

void MdbMasterTask::SetWord(uint8_t *p, uint16_t w) {
	p[0] = w >> 8;
	p[1] = w & 0xff;
}

uint16_t MdbMasterTask::GetWord(const uint8_t *p) {
	return (p[0] << 8) | p[1];
}

float MdbMasterTask::GetFloat(const uint8_t *p) {
	float w;
	uint16_t *pw = (uint16_t*) &w;
	pw[0] = GetWord(&p[0]);
	pw[1] = GetWord(&p[2]);
	return w;
}

//Holding Registers
void MdbMasterTask::sendMdbFun3(ReqSrc reqSrc, uint8_t DevNr, uint16_t regAdr, uint16_t regCnt) {
	state.sent.regAdr = regAdr;
	state.sent.regCnt = regCnt;

	sendBuf[0] = DevNr;
	sendBuf[1] = 3;
	SetWord(&sendBuf[2], regAdr - 1);
	SetWord(&sendBuf[4], regCnt);
	sendMdbFun(reqSrc, sendBuf, 6);
}

//Input Registers
void MdbMasterTask::sendMdbFun4(ReqSrc reqSrc, uint8_t DevNr, uint16_t regAdr, uint16_t regCnt) {
	state.sent.regAdr = regAdr;
	state.sent.regCnt = regCnt;

	sendBuf[0] = DevNr;
	sendBuf[1] = 4;
	SetWord(&sendBuf[2], regAdr - 1);
	SetWord(&sendBuf[4], regCnt);
	sendMdbFun(reqSrc, sendBuf, 6);
}

void MdbMasterTask::sendMdbFun6(ReqSrc reqSrc, uint8_t DevNr, uint16_t regAdr, uint16_t regVal) {
	state.sent.regAdr = regAdr;
	state.sent.regVal = regVal;

	sendBuf[0] = DevNr;
	sendBuf[1] = 6;
	SetWord(&sendBuf[2], regAdr - 1);
	SetWord(&sendBuf[4], regVal);
	sendMdbFun(reqSrc, sendBuf, 6);
}

void MdbMasterTask::sendMdbFun16(ReqSrc reqSrc, uint8_t DevNr, uint16_t regAdr, uint16_t regCnt, uint16_t *regTab) {
	if (regCnt < 120) {
		state.sent.regAdr = regAdr;
		state.sent.regCnt = regCnt;

		sendBuf[0] = DevNr;
		sendBuf[1] = 16;
		SetWord(&sendBuf[2], regAdr - 1);
		SetWord(&sendBuf[4], regCnt);
		sendBuf[6] = regCnt << 1;
		for (int i = 0; i < regCnt; i++) {
			SetWord(&sendBuf[7 + 2 * i], regTab[i]);
		}
		sendMdbFun(reqSrc, sendBuf, 7 + 2 * regCnt);
	}
}

void MdbMasterTask::buidMsgRec(MsgV *m) {
	m->err = (mDbgLevel >= 1) || (state.sent.currReq == reqCONSOLA);
	m->info = (mDbgLevel >= 2) || (state.sent.currReq == reqCONSOLA);
	m->dat = (mDbgLevel >= 3) || (state.sent.currReq == reqCONSOLA);
}

void MdbMasterTask::proceedRecFrame() {
	int n = mUart->getRxCharCnt();
	if (n > 4) {
		const uint8_t *inBuf = mUart->getRxBuf();
		MsgV m;
		buidMsgRec(&m);

		if (TCrc::Check(inBuf, n)) {
			if (inBuf[0] != state.sent.devNr) {
				if (m.err)
					shellTask->msg(colRED, "MDB%u: replay DEVNR not agree: %u<->%u", mMdbNr, state.sent.devNr, inBuf[0]);
			} else {
				uint8_t rxCmd = inBuf[1] & 0x7F;
				if (rxCmd != state.sent.code) {
					if (m.err)
						shellTask->msg(colRED, "MDB%u: replay FUN not agree: %u<->%u", mMdbNr, state.sent.code, rxCmd);
				} else {
					state.sent.currReq = reqEMPTY;
					if ((inBuf[1] & 0x80) == 0) {
						switch (rxCmd) {
						case 3:
						case 4: {
							int n = inBuf[2] >> 1;
							if (n != state.sent.regCnt) {
								if (m.err)
									shellTask->msg(colRED, "MDB%u: Fun%u REPLY ERROR", mMdbNr, rxCmd);
								onReciveData(false, rxCmd, NULL, 0);

							} else {
								if (m.dat) {
									char txt[200];
									int m = snprintf(txt, sizeof(txt), "MDB%u: %04X>", mMdbNr, state.sent.regAdr);
									for (int i = 0; i < n; i++) {
										uint16_t w = GetWord(&inBuf[3 + 2 * i]);
										m += snprintf(&txt[m], sizeof(txt) - m, "%02X,", w);
									}
									shellTask->msg(colWHITE, txt);
								}
								onReciveData(true, rxCmd, &inBuf[3], n);
							}

						}
							break;
						case 6: {
							uint16_t reg = GetWord(&inBuf[2]) + 1;
							uint16_t v = GetWord(&inBuf[4]);
							if ((reg == state.sent.regAdr) && (v == state.sent.regVal)) {
								if (m.info)
									shellTask->msg(colWHITE, "MDB%u: Fun6 ACK", mMdbNr);
							} else {
								if (m.err)
									shellTask->msg(colRED, "MDB%u: Fun6 ACK ERROR", mMdbNr);
							}
						}
							break;
						case 16: {
							uint16_t reg = GetWord(&inBuf[2]) + 1;
							uint16_t cnt = GetWord(&inBuf[4]);
							if ((reg == state.sent.regAdr) && (cnt == state.sent.regCnt)) {
								onReciveData(true, rxCmd, NULL, 0);
								if (m.info)
									shellTask->msg(colWHITE, "MDB%u: Fun16 ACK", mMdbNr);
							} else {
								onReciveData(false, rxCmd, NULL, 0);
								if (m.err)
									shellTask->msg(colRED, "MDB%u: Fun16 ACK ERROR", mMdbNr);
							}
						}
							break;
						}
					} else {
						onReciveData(false, rxCmd, NULL, 0);
						if (m.err)
							shellTask->msg(colRED, "MDB%u: Modbus exception %u", mMdbNr, inBuf[2]);
					}
				}

			}

		}
	}
	mUart->clearRxBuf();

}

void MdbMasterTask::sendConsolaReq() {
	if (state.sent.currReq == reqEMPTY) {
		switch (reqConsola.fun) {
		case 3:
			sendMdbFun3(reqCONSOLA, reqConsola.devNr, reqConsola.regAdr, reqConsola.regCnt);
			break;
		case 4:
			sendMdbFun4(reqCONSOLA, reqConsola.devNr, reqConsola.regAdr, reqConsola.regCnt);
			break;
		case 6:
			sendMdbFun6(reqCONSOLA, reqConsola.devNr, reqConsola.regAdr, reqConsola.regVal[0]);
			break;
		case 16:
			sendMdbFun16(reqCONSOLA, reqConsola.devNr, reqConsola.regAdr, reqConsola.regCnt, reqConsola.regVal);
			break;
		}
		reqConsola.fun = 0;
	}
}

void MdbMasterTask::ThreadFunc() {
	uint32_t sleepTm = 50;
	xEventGroupWaitBits(sysEvents, EVENT_CREATE_DEVICES, false, false, 1000000);

	switch (mMdbNr) {
	case MDB_1:
		mDbgLevel = config->data.R.rest.mdb1dbgLevel;
		break;
	case MDB_2:
		mDbgLevel = config->data.R.rest.mdb2dbgLevel;
		break;
	case MDB_3:
		mDbgLevel = config->data.R.rest.mdb3dbgLevel;
		break;

	}

	while (1) {
		imAlive();

		osEvent ev = osSignalWait(SIGNAL_RXCHAR | SIGNAL_CMD, sleepTm);
		sleepTm = 50;
		uint32_t tt = HAL_GetTick();

		MsgV m;
		buidMsgRec(&m);

//czy TimeOUT  dla funkcji Modbus
		if (state.sent.currReq != reqEMPTY) {
			if (tt - state.sent.tick > MAX_MDB_REPL_TM) {
				state.sent.currReq = reqEMPTY;
				state.timeOutCnt++;
				doOnTimeOut();
				if (m.err)
					shellTask->msg(colRED, "MDB%u: TimeOut DevNr=%u Fun=%u", mMdbNr, state.sent.devNr, state.sent.code);
			}
		}

		uint32_t tr = mUart->getLastRxCharTick();
		if (tr != 0 && tt - tr > 10) {
			proceedRecFrame();
		}

		if (ev.status == osEventSignal) {
			int code = ev.value.v;
			if (code & SIGNAL_RXCHAR) {
				sleepTm = 11;
			}

			if (code & SIGNAL_CMD) {
			}
		}
		loopFunc();

		if (reqConsola.fun != 0) {
			sendConsolaReq();
		}

	} // koniec pętli
}

void MdbMasterTask::showState(MsgStream *strm) {
	if (strm->msgOpen(colWHITE)) {
		strm->msgItem("PWR_ON=%u", getPower());
		strm->msgItem("PWR_FLT=%u", getPowerFlt());
		strm->msgItem("RxCnt=%u", mUart->getRxGlobCnt());
		strm->msgClose();
	}

}

const ShellItem menuMdb[] = { //
		{ "dbg", "zmien poziom logów (0..4)" }, //
				{ "s", "stan" }, //
				{ "pwr", "załaczenie zasilania +5V" }, //
				{ "rdreg", "read registers MDB3: devNr,Addr,cnt" }, //
				{ "rdinp", "read analog input MDB4: devNr,Addr,cnt" }, //
				{ "wrreg", "write register MDB6: devNr,Addr,val" }, //
				{ "wrmul", "write registers MDB16: devNr,Addr,val1,val2,..valX" }, //

				{ NULL, NULL } };

const ShellItem* MdbMasterTask::getMenu() {
	return menuMdb;
}

const char* MdbMasterTask::getMenuName() {
	return "Modbus Menu";
}

void MdbMasterTask::shell(MsgStream *strm, const char *cmd) {
	char tok[20];
	int idx = -1;

	const ShellItem *menu = getMenu();

	if (Token::get(&cmd, tok, sizeof(tok)))
		idx = findCmd(menu, tok);

	if (!execMenuItem(strm, idx, cmd)) {
		showHelp(strm, getMenuName(), menu);
	}
}

void MdbMasterTask::buildMenu(const ShellItem *toAddMenu) {
	const ShellItem *item = menuMdb;
	int k = 0;
	while (item->cmd != NULL) {
		k++;
		item++;
	}
	menu.baseCnt = k;
	item = toAddMenu;
	while (item->cmd != NULL) {
		k++;
		item++;
	}
	menu.tab = (ShellItem*) malloc((k + 1) * sizeof(ShellItem));

	k = 0;
	item = menuMdb;
	while (item->cmd != NULL) {
		menu.tab[k].cmd = item->cmd;
		menu.tab[k].descr = item->descr;
		k++;
		item++;
	}
	item = toAddMenu;
	while (item->cmd != NULL) {
		menu.tab[k].cmd = item->cmd;
		menu.tab[k].descr = item->descr;
		k++;
		item++;
	}
	menu.tab[k].cmd = NULL;
	menu.tab[k].descr = NULL;
}

bool MdbMasterTask::execMenuItem(MsgStream *strm, int idx, const char *cmd) {
	switch (idx) {
	case 0:  //dbg
		Token::getAsInt(&cmd, &mDbgLevel);
		break;
	case 1:  //s
		showState(strm);
		break;
	case 2: { //pwr
		bool q;
		Token::getAsBool(&cmd, &q);
		setPower(q);
		strm->msg(colWHITE, "SetPower=%u", q);
	}
		break;
	case 3: //rdreg
	case 4: { //rdinp
		int devNr;
		int adr;
		int cnt;
		if (Token::getAsInt(&cmd, &devNr)) {
			if (Token::getAsInt(&cmd, &adr)) {
				if (Token::getAsInt(&cmd, &cnt)) {
					reqConsola.devNr = devNr;
					reqConsola.regAdr = adr;
					reqConsola.regCnt = cnt;
					const char *nm;
					if (idx == 3) {
						reqConsola.fun = 3;
						nm = "RdReg";
					} else {
						reqConsola.fun = 4;
						nm = "RdInp";
					}
					strm->msg(colWHITE, "DevNr=%u %s %u,%u", reqConsola.devNr, nm, reqConsola.regAdr, reqConsola.regCnt);
					osSignalSet(getThreadId(), SIGNAL_CMD);
				}
			}
		}
	}
		break;

	case 5: { //wrreg
		int devNr;
		int adr;
		int val;
		if (Token::getAsInt(&cmd, &devNr)) {
			if (Token::getAsInt(&cmd, &adr)) {
				if (Token::getAsInt(&cmd, &val)) {
					reqConsola.fun = 6;
					reqConsola.devNr = devNr;
					reqConsola.regAdr = adr;
					reqConsola.regVal[0] = val;
					strm->msg(colWHITE, "DevNr=%u WrReg %u: %u", reqConsola.devNr, reqConsola.regAdr, reqConsola.regVal[0]);
					osSignalSet(getThreadId(), SIGNAL_CMD);
				}
			}
		}
	}
		break;
	case 6: { //wrmul
		int devNr;
		int adr;
		if (Token::getAsInt(&cmd, &devNr)) {
			if (Token::getAsInt(&cmd, &adr)) {
				int n = 0;
				while (n < MAX_VAL_CNT) {
					int val;
					if (Token::getAsInt(&cmd, &val)) {
						reqConsola.regVal[n] = val;
						n++;
					} else
						break;
				}
				if (n > 0) {
					reqConsola.fun = 16;
					reqConsola.devNr = devNr;
					reqConsola.regAdr = adr;
					reqConsola.regCnt = n;
					strm->msg(colWHITE, "DevNr=%u WrMulReg %u: n=%u", reqConsola.devNr, reqConsola.regAdr, n);
					osSignalSet(getThreadId(), SIGNAL_CMD);
				}
			}
		}
	}
		break;
	default:
		return false;
	};
	return true;
}

//-----------------------------------------------------------------------------------------
// FiltrIR
//-----------------------------------------------------------------------------------------
FiltrIR::FiltrIR(float k) {
	firstDt = true;
	state = 0;
	mK = k;
}

void FiltrIR::inp(float x) {
	if (firstDt) {
		firstDt = false;
		state = x;
	} else {
		state = (1 - mK) * state + mK * x;
	}
}

float FiltrIR::out() {
	return state;
}

//-----------------------------------------------------------------------------------------
// FiltrFIR
//-----------------------------------------------------------------------------------------

FiltrFIR::FiltrFIR(int len) {
	memset(tab, 0, sizeof(tab));
	mOverride = false;
	mPtr = 0;
	mLen = len;
	if (mLen > MAX_LEN || mLen < 1)
		mLen = MAX_LEN;
}
void FiltrFIR::inp(float x) {
	tab[mPtr] = x;
	if (++mPtr >= MAX_LEN) {
		mPtr = 0;
		mOverride = true;
	}
}

float FiltrFIR::out() {
	float sum = 0;
	int k = 0;
	int ptr = mPtr;
	while (k < mLen) {
		if (ptr == 0) {
			if (!mOverride) {
				break;
			}
			ptr = MAX_LEN;
		}
		ptr--;
		sum += tab[ptr];
		k++;
	}
	if (k > 0)
		return sum / k;
	else
		return NAN;
}

//-----------------------------------------------------------------------------------------
// MdbMasterGasTask
//-----------------------------------------------------------------------------------------
MdbMasterGasTask::MdbMasterGasTask(int mdbNr, int portNr) :
		MdbMasterTask::MdbMasterTask(mdbNr, portNr) {
	memset(&autoRd, 0, sizeof(autoRd));
	memset(&gasData, 0, sizeof(gasData));

	strcpy(autoRd.statusTxt, "Start");
	zeroOfs.flag = false;
	zeroOfs.cnt = 0;
	zeroOfs.phase = 0;
	zeroOfs.sensNr = 0;
}

bool MdbMasterGasTask::isCfgAnyGas() {
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

float MdbMasterGasTask::getGasFactor(int id) {
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
	}
	return 0;
}

void MdbMasterGasTask::onReciveData(bool replOK, uint8_t mdbFun, const uint8_t *tab, int regCnt) {
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

					sensor->Typ = id >> 8;
					sensor->VerTyp = id & 0xff;
					sensor->Status = GetWord(&tab[base + 2]);
					int v = GetWord(&tab[base + 4]);
					int vh = GetWord(&tab[base + 6]);
					v = v | (vh << 16);
					sensor->valueHd = v;
					float factor = getGasFactor(sensor->Typ);
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
		shellTask->msg(colWHITE, "MDB%u: Ack %s. Phase=%u sensNr=%u", mMdbNr, ErrOk(!replOK), zeroOfs.phase, zeroOfs.sensNr);
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
					shellTask->msg(colGREEN, "MDB%u: Send zero offset OK", mMdbNr);
					zeroOfs.phase = 0;
				}
				break;
			}
		} else {
			shellTask->msg(colRED, "MDB%u: Send zero offset ERROR. Phase=%u sensNr=%u", mMdbNr, zeroOfs.phase, zeroOfs.sensNr);
			zeroOfs.phase = 0;
		}
	}
}

void MdbMasterGasTask::doOnTimeOut() {
	if (zeroOfs.phase > 0) {
		zeroOfs.phase = 0;
		shellTask->msg(colRED, "MDB%u: Send zero offset TIMEOUT. Phase=%u sensNr=%u", mMdbNr, zeroOfs.phase, zeroOfs.sensNr);
	} else if (autoRd.phase > 0) {
		autoRd.phase = 0;
		shellTask->msg(colRED, "MDB%u: read measure TIMEOUT", mMdbNr);
		strcpy(autoRd.statusTxt, "TimeOut");
	} else {
		shellTask->msg(colRED, "MDB%u: TIMEOUT", mMdbNr);
	}
}

void MdbMasterGasTask::sendZeroOfs_FrameZero(uint16_t *tab) {
	tab[0] = 0x30c6;
	tab[1] = 0x431b;
	tab[2] = 0x0000;
	tab[3] = 0x0000;
	tab[4] = 0x0000;
	tab[5] = 0x0000;
	tab[6] = 0x0000;
}

void MdbMasterGasTask::sendZeroOfs_Phase1Frame() {
	uint16_t tab[7];

	sendZeroOfs_FrameZero(tab);
	tab[6] = zeroOfs.sensNr * 0x100;
	sendMdbFun16(reqSYS, config->data.R.rest.gasDevMdbNr, 10001, 7, tab);
}

void MdbMasterGasTask::sendZeroOfs_Phase5Frame() {
	uint16_t tab[7];

	sendZeroOfs_FrameZero(tab);
	tab[6] = zeroOfs.sensNr * 0x100 + 3;
	sendMdbFun16(reqSYS, config->data.R.rest.gasDevMdbNr, 10001, 7, tab);
}

void MdbMasterGasTask::sendZeroOfs_Phase3Frame() {
	uint16_t tab[7];

	sendZeroOfs_FrameZero(tab);
	for (int i = 0; i < gasData.devCntTh; i++) {
		tab[2 + i] = zeroOfs.tab[i];
	}
	tab[6] = 0x0005;
	sendMdbFun16(reqSYS, config->data.R.rest.gasDevMdbNr, 10001, 7, tab);
}

void MdbMasterGasTask::loopFunc() {
	if (isCfgAnyGas()) {
		if (HAL_GetTick() - autoRd.tick > TM_AUTO_RD) {
			if (zeroOfs.phase == 0) {
				autoRd.tick = HAL_GetTick();
				autoRd.phase = 1;
			}
		}
		if (autoRd.phase != 0) {
			if (state.sent.currReq == reqEMPTY) {
				switch (autoRd.phase) {
				case 1:
					autoRd.reqCnt++;
					autoRd.phase = 2;
					sendMdbFun4(reqSYS, config->data.R.rest.gasDevMdbNr, 1, 5);
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
		if (zeroOfs.flag) {
			if (autoRd.phase == 0) {
				zeroOfs.flag = false;
				zeroOfs.phase = 1;
				zeroOfs.sensNr = 0;
				shellTask->msg(colYELLOW, "ZeroOfs  cnt=%u", zeroOfs.cnt);
			}
		}
		if (zeroOfs.phase != 0) {
			if (state.sent.currReq == reqEMPTY) {
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
					if (HAL_GetTick() - state.sent.tick > MAX_TIME_REPL + 500)
						zeroOfs.phase = 0;
					break;
				}

			}
		}
	}
}

void MdbMasterGasTask::showState(MsgStream *strm) {
	MdbMasterTask::showState(strm);
	if (strm->msgOpen(colWHITE)) {
		strm->msgItem("RedCnt=%d", autoRd.redCnt);
		strm->msgItem("TmRd=%.2f[s]", (HAL_GetTick() - autoRd.redTick) / 1000.0);
		strm->msgClose();

	}
}

const char* MdbMasterGasTask::getSenName(int typ, int verTyp) {
	static char txt[20];
	switch (typ) {
	case 1:
		strcpy(txt, "NO2");
		break;
	case 2:
		strcpy(txt, "O3 ");
		break;
	case 3:
		strcpy(txt, "CO ");
		break;
	case 4:
		strcpy(txt, "SO2");
		break;
	default:
		strcpy(txt, "??");
		break;
	}
	int n = strlen(txt);
	sprintf(&txt[n], "(v%d)", verTyp);
	return txt;
}

bool MdbMasterGasTask::isMeasValid(uint16_t status) {
	return ((status & 0x07) == 0);
}

const char* MdbMasterGasTask::getSensValidStr(uint16_t status) {
	if ((status & 0x07) != 0)
		return "ERR";
	else if ((status & 0x3FF) != 0)
		return "WARN";
	else
		return "OK";
}

void MdbMasterGasTask::showMeas(MsgStream *strm) {
	if (strm->msgOpen(colWHITE)) {
		strm->msgItem("RedCnt=%d", autoRd.redCnt);
		strm->msgItem("TmRd=%.2f[s]", (HAL_GetTick() - autoRd.redTick) / 1000.0);
		strm->msgItem("DevCnt=%d (%d)", gasData.devCnt, gasData.devCntTh);
		strm->msgItem("serialNum=%d", gasData.serialNum);
		strm->msgItem("ProdYear=%d", gasData.ProdYear);
		strm->msgItem("FirmwareVer=%d.%03d", gasData.FirmwareVer >> 8, gasData.FirmwareVer & 0xff);
		strm->msgItem("FailureCode=%d", gasData.FailureCode);

		SensorData *pD = gasData.sensorTab;
		for (int i = 0; i < gasData.devCntTh; i++) {
			int n = snprintf(gtxt, sizeof(gtxt), "%u. %s %s Status=0x%04X ", i + 1, //
			getSenName(pD->Typ, pD->VerTyp), getSensValidStr(pD->Status), pD->Status);

			if (isMeasValid(pD->Status)) {
				n += snprintf(&gtxt[n], sizeof(gtxt) - n, "v=%d[ppb]  %.3f[ug/m3]", pD->valueHd, pD->valueFiz);
				if (pD->filtrFIR != NULL) {
					n += snprintf(&gtxt[n], sizeof(gtxt) - n, " (FIR:%.3f) (IR:%.3f)", pD->filtrFIR->out(), pD->filtrIR->out());
				}
			}
			strm->msgItem(gtxt);
			pD++;

		}

		strm->msgClose();
	}
}

bool MdbMasterGasTask::isError() {
	return ((autoRd.redTick == 0) || (HAL_GetTick() - autoRd.redTick > TIME_MEAS_VALID));
}

bool MdbMasterGasTask::getGasValue(MeasType measType, float *val) {
	return getGasValue(measType, config->data.R.exDev.gasFiltrType, val);
}

bool MdbMasterGasTask::getGasValue(MeasType measType, int filtrType, float *val) {

	if (isError()) {
		*val = NAN;
		return false;
	}

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
		int code = gasData.sensorTab[i].Typ;
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

const ShellItem menuGas[] = { //
		{ "m", "pomiary" }, //
				{ "zero", "ustawienie zera" }, //

				{ NULL, NULL } };

const ShellItem* MdbMasterGasTask::getMenu() {
	if (menu.tab == NULL) {
		buildMenu(menuGas);
	}
	return menu.tab;
}

bool MdbMasterGasTask::zeroGasFromSMS(const char *ptr, char *resText, int maxLen) {
	zeroOfs.cnt = 0;
	while (zeroOfs.cnt < MAX_DEV_CNT) {
		int a;
		if (Token::getAsInt(&ptr, &a)) {
			zeroOfs.tab[zeroOfs.cnt++] = a;
		} else
			break;
	}
	if (zeroOfs.cnt == gasData.devCntTh) {
		if (shellTask->msgOpen(colBLUE)) {
			shellTask->msgItem("GAS: Zero offset z SMS");
			for (int i = 0; i < gasData.devCntTh; i++) {
				shellTask->msgItem("%u. z=%d", i + 1, (int) ((int16_t) zeroOfs.tab[i]));
			}
			shellTask->msgClose();
		}

		zeroOfs.flag = true;
		strlcpy(resText, "ZERO-GAS. Procedura rozpoczęta.", maxLen);
		return true;
	} else {
		shellTask->msg(colBLUE, "SMS Błąd. Ilość parametrów powinna być %u", gasData.devCntTh);
		snprintf(resText, maxLen, "ZERO-GAS. Błąd, ilość parametrów powinna być %u", gasData.devCntTh);
		return false;
	}
}

bool MdbMasterGasTask::execMyMenuItem(MsgStream *strm, int idx, const char *cmd) {
	switch (idx) {
	case 0:  //m
		showMeas(strm);
		break;
	case 1: {  //zero
		strm->msg(colWHITE, "Zero offset");
		zeroOfs.cnt = 0;
		while (zeroOfs.cnt < MAX_DEV_CNT) {
			int a;
			if (Token::getAsInt(&cmd, &a)) {
				zeroOfs.tab[zeroOfs.cnt++] = a;
			} else
				break;
		}
		if (zeroOfs.cnt == gasData.devCntTh) {
			zeroOfs.flag = true;
		} else {
			strm->msg(colYELLOW, "Błąd. Ilość parametrów powinna być %u", gasData.devCntTh);
		}

	}
		break;
	default:
		return false;
	}
	return true;
}

bool MdbMasterGasTask::execMenuItem(MsgStream *strm, int idx, const char *cmd) {
	bool q = MdbMasterTask::execMenuItem(strm, idx, cmd);
	if (!q) {
		q = execMyMenuItem(strm, idx - menu.baseCnt, cmd);
	}
	return q;
}

const char* MdbMasterGasTask::getMenuName() {
	return "GAS sensor Menu";
}

void MdbMasterGasTask::getKomoraStatusTxt(char *txt, int max) {
	snprintf(txt, max, "ReqCnt=%u RdCnt=%u TimeOutCnt=%u ST=%s", autoRd.reqCnt, autoRd.redCnt, state.timeOutCnt, autoRd.statusTxt);
}

//-----------------------------------------------------------------------------------------
// MdbMasterNoiseTask
//-----------------------------------------------------------------------------------------
MdbMasterNoiseTask::MdbMasterNoiseTask(int mdbNr, int portNr) :
		MdbMasterTask::MdbMasterTask(mdbNr, portNr) {
	memset(&autoRd, 0, sizeof(autoRd));
	memset(&noiseData, 0, sizeof(noiseData));

	strcpy(autoRd.statusTxt, "Start");
	menu.tab = NULL;
	menu.baseCnt = 0;
}

bool MdbMasterNoiseTask::isCfgNoiseOn() {
	return (config->data.R.exDev.sensExist[ssNOISE]);
}

void MdbMasterNoiseTask::onReciveData(bool replOK, uint8_t mdbFun, const uint8_t *tab, int regCnt) {
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

void MdbMasterNoiseTask::doOnTimeOut() {
	if (autoRd.phase > 0) {
		autoRd.phase = 0;
		shellTask->msg(colRED, "MDB%u: read measure TIMEOUT", mMdbNr);
		strcpy(autoRd.statusTxt, "TimeOut");
	} else {
		shellTask->msg(colRED, "MDB%u: TIMEOUT", mMdbNr);
	}
}

void MdbMasterNoiseTask::loopFunc() {
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

void MdbMasterNoiseTask::showState(MsgStream *strm) {
	MdbMasterTask::showState(strm);
	if (strm->msgOpen(colWHITE)) {
		strm->msgItem("RedCnt=%d", autoRd.redCnt);
		strm->msgItem("TmRd=%.2f[s]", (HAL_GetTick() - autoRd.redTick) / 1000.0);
		strm->msgClose();

	}
}

void MdbMasterNoiseTask::showMeas(MsgStream *strm) {
	if (strm->msgOpen(colWHITE)) {
		strm->msgItem("ReqCnt=%d", autoRd.reqCnt);
		strm->msgItem("RedCnt=%d", autoRd.redCnt);
		strm->msgItem("TmRd=%.2f[s]", (HAL_GetTick() - autoRd.redTick) / 1000.0);

		int n = snprintf(gtxt, sizeof(gtxt), "Noise: ");

		if (!isError()) {
			n += snprintf(&gtxt[n], sizeof(gtxt) - n, "%.1f[dB]", noiseData.valueFiz);
			if (noiseData.filtrFIR != NULL) {
				n += snprintf(&gtxt[n], sizeof(gtxt) - n, " (FIR:%.1f) (IR:%.1f)", noiseData.filtrFIR->out(), noiseData.filtrIR->out());
			}
		} else {
			n += snprintf(&gtxt[n], sizeof(gtxt) - n, "brak danych");
		}
		strm->msgItem(gtxt);
	}

	strm->msgClose();

}

bool MdbMasterNoiseTask::isError() {
	return ((autoRd.redTick == 0) || (HAL_GetTick() - autoRd.redTick > TIME_MEAS_VALID));
}

bool MdbMasterNoiseTask::getNoiseValue(float *val) {
	return getNoiseValue(config->data.R.exDev.noiseFiltrType, val);
}

bool MdbMasterNoiseTask::getNoiseValue(int filtrType, float *val) {

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

const ShellItem* MdbMasterNoiseTask::getMenu() {
	if (menu.tab == NULL) {
		buildMenu(menuNoise);
	}
	return menu.tab;
}

bool MdbMasterNoiseTask::execMyMenuItem(MsgStream *strm, int idx, const char *cmd) {
	switch (idx) {
	case 0:  //m
		showMeas(strm);
		break;
	default:
		return false;
	}
	return true;
}

bool MdbMasterNoiseTask::execMenuItem(MsgStream *strm, int idx, const char *cmd) {
	bool q = MdbMasterTask::execMenuItem(strm, idx, cmd);
	if (!q) {
		q = execMyMenuItem(strm, idx - menu.baseCnt, cmd);
	}
	return q;
}

const char* MdbMasterNoiseTask::getMenuName() {
	return "Noise sensor Menu";
}

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
								shellTask->msg(colGREEN, "MDB%u:T=%u SetHeater:%u temp=%.1f[*C]", mMdbNr, autoRd.heaterOrderLastSendTick, doOn, dustData.temperature);
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
		shellTask->msg(colRED, "MDB%u: read dust measure TIMEOUT", mMdbNr);
		strcpy(autoRd.statusTxt, "TimeOut");
	} else {
		shellTask->msg(colRED, "MDB%u: TIMEOUT", mMdbNr);
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

void MdbMasterDustTask::showState(MsgStream *strm) {
	MdbMasterTask::showState(strm);
	if (strm->msgOpen(colWHITE)) {
		strm->msgItem("RdTime=%.2f[s]", (float) ((HAL_GetTick() - autoRd.redTick)) / 1000.0);
		strm->msgItem("autoRd.Status=%s", autoRd.statusTxt);
		strm->msgItem("measValid=%u", isMeasValid());
		strm->msgItem("autoRd.reqCnt=%u", autoRd.reqCnt);
		strm->msgItem("autoRd.redCnt=%u", autoRd.redCnt);
		strm->msgClose();
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
void MdbMasterDustTask::showMeas(MsgStream *strm) {
	if (strm->msgOpen(colWHITE)) {
		strm->msgItem("DevId=0x%04X", dustData.devID);
		strm->msgItem("SerNum=%u", dustData.serialNumer);
		strm->msgItem("prodYear=%u", dustData.productYear);
		strm->msgItem("FirmVer=%u.%03u", dustData.firmware.ver, dustData.firmware.rev);
		strm->msgItem("FailureCode=0x%04X", dustData.failureCode);
		strm->msgItem("Temperature=%.1f", dustData.temperature);
		strm->msgItem("HeaterOn=%u", dustData.heaterOn);
		strm->msgItem("PM_Status=0x%04X", dustData.PmStatus);
		if (isMeasValid()) {
			strm->msgItem("PM 1.0= %.1f[ug/m3]", dustData.pm1_0);
			strm->msgItem("PM 2.5= %.1f[ug/m3]", dustData.pm2_5);
			strm->msgItem("PM 4.0= %.1f[ug/m3]", dustData.pm4_0);
			strm->msgItem("PM 10 = %.1f[ug/m3]", dustData.pm10);
			strm->msgItem("DustCnt'0_5'=%.2f[#/cm3]", dustData.dustCnt0_5);
			strm->msgItem("DustCnt'1_0'=%.2f[#/cm3]", dustData.dustCnt1_0);
			strm->msgItem("DustCnt'2_5'=%.2f[#/cm3]", dustData.dustCnt2_5);
			strm->msgItem("DustCnt'4_0'=%.2f[#/cm3]", dustData.dustCnt4_0);
			strm->msgItem("DustCnt'10' =%.2f[#/cm3]", dustData.dustCnt10);
			strm->msgItem("DustSize =%.2f[nm]", dustData.dustSize);
		}
		strm->msgClose();
	}
}

const ShellItem menuExternDust[] = { //
		{ "m", "pomiary" }, //
				{ "heater_on", "włączenie grzania" }, //
				{ "heater_off", "wyłączenie grzania" }, //

				{ NULL, NULL } };

const ShellItem* MdbMasterDustTask::getMenu() {
	if (menu.tab == NULL) {
		buildMenu(menuExternDust);
	}
	return menu.tab;
}

void MdbMasterDustTask::setHeater(ReqSrc reqSrc, bool heaterOn) {
	uint16_t w = 0;
	if (heaterOn)
		w = HEATER_CONST_ON;
	sendMdbFun6(reqSrc, config->data.R.rest.dustDevMdbNr, 8, w);
}

bool MdbMasterDustTask::execMyMenuItem(MsgStream *strm, int idx, const char *cmd) {
	switch (idx) {
	case 0:  //m
		showMeas(strm);
		break;
	case 1:  //heater_on
		setHeater(reqCONSOLA, true);
		break;
	case 2:  //heater_off
		setHeater(reqCONSOLA, false);
		break;
	default:
		return false;
	}
	return true;
}

bool MdbMasterDustTask::execMenuItem(MsgStream *strm, int idx, const char *cmd) {
	bool q = MdbMasterTask::execMenuItem(strm, idx, cmd);
	if (!q) {
		q = execMyMenuItem(strm, idx - menu.baseCnt, cmd);
	}
	return q;
}

const char* MdbMasterDustTask::getMenuName() {
	return "Extern Dust Menu";
}

bool MdbMasterDustTask::isError() {
	return ((autoRd.redTick == 0) || (HAL_GetTick() - autoRd.redTick > TIME_MEAS_VALID));
}
