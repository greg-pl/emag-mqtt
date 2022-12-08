/*
 * MdbMaster.cpp
 *
 *  Created on: 6 lut 2021
 *      Author: Grzegorz
 */
#include <MdbMasterTask.h>

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
					shellTask->oMsgX(colRED, "MDB%u: replay DEVNR not agree: %u<->%u", mMdbNr, state.sent.devNr, inBuf[0]);
			} else {
				uint8_t rxCmd = inBuf[1] & 0x7F;
				if (rxCmd != state.sent.code) {
					if (m.err)
						shellTask->oMsgX(colRED, "MDB%u: replay FUN not agree: %u<->%u", mMdbNr, state.sent.code, rxCmd);
				} else {
					state.sent.currReq = reqEMPTY;
					if ((inBuf[1] & 0x80) == 0) {
						switch (rxCmd) {
						case 3:
						case 4: {
							int n = inBuf[2] >> 1;
							if (n != state.sent.regCnt) {
								if (m.err)
									shellTask->oMsgX(colRED, "MDB%u: Fun%u REPLY ERROR", mMdbNr, rxCmd);
								onReciveData(false, rxCmd, NULL, 0);

							} else {
								if (m.dat) {
									char txt[200];
									int m = snprintf(txt, sizeof(txt), "MDB%u: %04X>", mMdbNr, state.sent.regAdr);
									for (int i = 0; i < n; i++) {
										uint16_t w = GetWord(&inBuf[3 + 2 * i]);
										m += snprintf(&txt[m], sizeof(txt) - m, "%02X,", w);
									}
									shellTask->oMsgX(colWHITE, txt);
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
									shellTask->oMsgX(colWHITE, "MDB%u: Fun6 ACK", mMdbNr);
							} else {
								if (m.err)
									shellTask->oMsgX(colRED, "MDB%u: Fun6 ACK ERROR", mMdbNr);
							}
						}
							break;
						case 16: {
							uint16_t reg = GetWord(&inBuf[2]) + 1;
							uint16_t cnt = GetWord(&inBuf[4]);
							if ((reg == state.sent.regAdr) && (cnt == state.sent.regCnt)) {
								onReciveData(true, rxCmd, NULL, 0);
								if (m.info)
									shellTask->oMsgX(colWHITE, "MDB%u: Fun16 ACK", mMdbNr);
							} else {
								onReciveData(false, rxCmd, NULL, 0);
								if (m.err)
									shellTask->oMsgX(colRED, "MDB%u: Fun16 ACK ERROR", mMdbNr);
							}
						}
							break;
						}
					} else {
						onReciveData(false, rxCmd, NULL, 0);
						if (m.err)
							shellTask->oMsgX(colRED, "MDB%u: Modbus exception %u", mMdbNr, inBuf[2]);
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
					shellTask->oMsgX(colRED, "MDB%u: TimeOut DevNr=%u Fun=%u", mMdbNr, state.sent.devNr, state.sent.code);
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

void MdbMasterTask::showState(OutStream *strm) {
	if (strm->oOpen(colWHITE)) {
		strm->oMsg("PWR_ON=%u", getPower());
		strm->oMsg("PWR_FLT=%u", getPowerFlt());
		strm->oMsg("RxCnt=%u", mUart->getRxGlobCnt());
		strm->oClose();
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

void MdbMasterTask::shell(OutStream *strm, const char *cmd) {
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

bool MdbMasterTask::execMenuItem(OutStream *strm, int idx, const char *cmd) {
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
		strm->oMsgX(colWHITE, "SetPower=%u", q);
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
					strm->oMsgX(colWHITE, "DevNr=%u %s %u,%u", reqConsola.devNr, nm, reqConsola.regAdr, reqConsola.regCnt);
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
					strm->oMsgX(colWHITE, "DevNr=%u WrReg %u: %u", reqConsola.devNr, reqConsola.regAdr, reqConsola.regVal[0]);
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
					strm->oMsgX(colWHITE, "DevNr=%u WrMulReg %u: n=%u", reqConsola.devNr, reqConsola.regAdr, n);
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




