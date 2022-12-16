/*
 * Bg96Driver.cpp
 *
 *  Created on: Dec 16, 2020
 *      Author: Grzegorz
 */

#include "stdint.h"
#include "string.h"
#include "stdarg.h"
#include "math.h"

#include "stm32f4xx_hal.h"

#include <Bg96Driver.h>
#include <main.h>
#include <UMain.h>
#include <Utils.h>
#include <cpx.h>
#include <GlobData.h>
#include "MdbMasterTask.h"
#include "AirDetRs.h"
#include "Token.h"

extern AirDetRs *airDetRs;

//-------------------------------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------------------------------
extern "C" const char* getRegStatusAsName(RegStatus regStatus) {
	switch (regStatus) {
	case regERROR:
		return "Blad rejestracji";
	case regNO_REG:
		return "No registred";
	case regREG_HOME:
		return "Registred to home network";
	case regREG_DENIED:
		return "Registred denied";
	case regREG_UNKNOWN:
		return "Unknow status";
	case regREG_ROAMING:
		return "Registred to roaming network";
	default:
	case regREG_BADREPLAY:
		return "Unknow replay";
	}
}

//-------------------------------------------------------------------------------------------------------------------------
// Bg96Uart
//-------------------------------------------------------------------------------------------------------------------------
Bg96Uart::Bg96Uart(int PortNr) :
		TUart::TUart(PortNr, 7) {
	mThreadId = NULL;
	inpStrem = new RxTxBuf(1024);
	memset(&stat, 0, sizeof(stat));
	mUseRts = true;
}

#define FLG(nm) offsetof(ReciveRec, nm)

HAL_StatusTypeDef Bg96Uart::Init() {
	HAL_StatusTypeDef st = TUart::Init(115200);
	if (st == HAL_OK) {
		HAL_UART_Receive_IT(&mHuart, (uint8_t*) &rxChar, 1);
	}
	return st;
}

//wywoływane z przerwania
void Bg96Uart::TxCpltCallback() {
	TUart::TxCpltCallback();
	if (mThreadId != NULL) {
		osSignalSet(mThreadId, Bg96Driver::SIGNAL_TXCPL);
	}
}

//wywoływane z przerwania
void Bg96Uart::RxCpltCallback() {
	inpStrem->add(rxChar);
	stat.rxCnt++;
	if (mThreadId != NULL) {
		osSignalSet(mThreadId, Bg96Driver::SIGNAL_CHAR);
		if (rxChar == '\n')
			osSignalSet(mThreadId, Bg96Driver::SIGNAL_CHAR_NL);
	}
	HAL_UART_Receive_IT(&mHuart, (uint8_t*) &rxChar, 1);
}

//wywoływane z wątku BgDriver
bool Bg96Uart::wiatForSent() {
	if (!isSending())
		return true;

	osEvent ev = osSignalWait(Bg96Driver::SIGNAL_TXCPL, 1000);
	if (ev.status == osEventSignal) {
		int code = ev.value.v;
		if (code == Bg96Driver::SIGNAL_TXCPL) {
			return true;
		}
	}
	return false;
}

void Bg96Uart::writeBuf(const char *txt, int len) {
	if (len < MAX_AT_LINE_SIZE) {
		memcpy(txBuf, txt, len);
		wiatForSent();
		TUart::writeBuf(txt, len);
	} else {
		stat.toBigLineCnt++;
	}

}

void Bg96Uart::writeStrNoCpy(const char *txt) {
	int len = strlen(txt);
	wiatForSent();
	TUart::writeBuf(txt, len);

}

void Bg96Uart::writeStr(const char *txt, bool andNl) {
	int len = strlen(txt);

	if (len < MAX_AT_LINE_SIZE - 3) {
		memcpy(txBuf, txt, len);
		if (andNl) {
			strcpy(&txBuf[len], "\r\n");
			len += 2;
		}
		wiatForSent();
		TUart::writeBuf(txBuf, len);
	} else {
		stat.toBigLineCnt++;
	}

}

void Bg96Uart::clearInpStream() {
	inpStrem->clear();
}

bool Bg96Uart::readLn(char *buf, int max) {
	bool q = inpStrem->readLn(buf, max);
	if (q) {
		stat.lnCnt++;
	}
	return q;
}

//-------------------------------------------------------------------------------------------------------------------------
// ShellTask
//-------------------------------------------------------------------------------------------------------------------------

Bg96Driver::Bg96Driver() :
		TaskClass::TaskClass("Bg96", osPriorityNormal, 0x600) {

	uart = new Bg96Uart(TUart::myUART2);
	loopCnt = 0;
	memset(&asynch, 0, sizeof(asynch));
	zeroState();

	mEcho.rx = false;
	mEcho.tx = false;
	mEcho.timeMode = 1;
	mEcho.logV = vvOFF;

	setEchoMode(config->data.R.bg96.BgEcho);

}

void Bg96Driver::zeroState() {
	memset(&bgParam, 0, sizeof(bgParam));
	memset(&state, 0, sizeof(state));
	state.gps.latitude = NAN;
	state.gps.longitude = NAN;
	state.rssi.rssi = NAN;
	clearRecDt();
}

void Bg96Driver::clearRecDt() {
	memset(&recR, 0, sizeof(recR));
}

void Bg96Driver::set3_8V(bool flag) {
	if (flag) {
		HAL_GPIO_WritePin(ON_OFF_3_8V_GPIO_Port, ON_OFF_3_8V_Pin, GPIO_PIN_RESET);
	} else {
		HAL_GPIO_WritePin(ON_OFF_3_8V_GPIO_Port, ON_OFF_3_8V_Pin, GPIO_PIN_SET);
	}
}
bool Bg96Driver::get3_8V() {
	return (HAL_GPIO_ReadPin(ON_OFF_3_8V_GPIO_Port, ON_OFF_3_8V_Pin) == GPIO_PIN_RESET);
}

void Bg96Driver::setReset(bool flag) {
	if (flag) {
		HAL_GPIO_WritePin(GSM_RESET_GPIO_Port, GSM_RESET_Pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(GSM_RESET_GPIO_Port, GSM_RESET_Pin, GPIO_PIN_RESET);
	}
}
bool Bg96Driver::getReset() {
	return (HAL_GPIO_ReadPin(GSM_RESET_GPIO_Port, GSM_RESET_Pin) == GPIO_PIN_SET);
}

void Bg96Driver::setPowerKey(bool flag) {
	if (flag) {
		HAL_GPIO_WritePin(GSM_ON_OFF_GPIO_Port, GSM_ON_OFF_Pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(GSM_ON_OFF_GPIO_Port, GSM_ON_OFF_Pin, GPIO_PIN_RESET);
	}
}
bool Bg96Driver::getPowerKey() {
	return (HAL_GPIO_ReadPin(GSM_ON_OFF_GPIO_Port, GSM_ON_OFF_Pin) == GPIO_PIN_SET);
}

bool Bg96Driver::getApReady() {
	return (HAL_GPIO_ReadPin(GSM_AP_RDY_GPIO_Port, GSM_AP_RDY_Pin) == GPIO_PIN_SET);
}

bool Bg96Driver::getHdStatus() {
	return (HAL_GPIO_ReadPin(GSM_STATUS_IN_GPIO_Port, GSM_STATUS_IN_Pin) == GPIO_PIN_SET);
}
bool Bg96Driver::getDCD() {
	return (HAL_GPIO_ReadPin(GSM_UART1_DCD_GPIO_Port, GSM_UART1_DCD_Pin) == GPIO_PIN_SET);
}

bool Bg96Driver::getRI() {
	return (HAL_GPIO_ReadPin(GSM_UART1_RI_GPIO_Port, GSM_UART1_RI_Pin) == GPIO_PIN_SET);
}

void Bg96Driver::setDTR(bool flag) {
	if (flag) {
		HAL_GPIO_WritePin(GSM_UART1_DTR_GPIO_Port, GSM_UART1_DTR_Pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(GSM_UART1_DTR_GPIO_Port, GSM_UART1_DTR_Pin, GPIO_PIN_RESET);
	}
}
bool Bg96Driver::getDTR() {
	return (HAL_GPIO_ReadPin(GSM_UART1_DTR_GPIO_Port, GSM_UART1_DTR_Pin) == GPIO_PIN_SET);
}

void Bg96Driver::logT(LogLevel lev, const char *frm, ...) {
	if (lev <= mEcho.logV) {
		TermColor color;
		switch (lev) {
		case vvERROR:
			color = colRED;
			break;
		default:
		case vvWARN:
			color = colYELLOW;
			break;
		case vvOFF:
		case vvHINT:
		case vvTEXT:
			color = colWHITE;
			break;

		}
		va_list ap;
		va_start(ap, frm);
		getOutStream()->oFormatX(color, frm, ap);
		va_end(ap);
	}
}

void Bg96Driver::clearBreakSignal() {
	osSignalWait(SIGNAL_BREAK, 1); //skasowanie sygnału
}

//delay with break
int Bg96Driver::dwb(uint32_t tm) {
	int tt = HAL_GetTick();
	while (HAL_GetTick() - tt < tm) {
		osDelay(5);
		if (asynch.doStop)
			return stBREAK;
	}
	return stOK;

}

void Bg96Driver::writeCmdNoEcho(const char *cmd, bool doDataCpy) {
	if (doDataCpy)
		uart->writeStr(cmd, false);
	else
		uart->writeStrNoCpy(cmd);
}

void Bg96Driver::writeCmd(const char *cmd) {
	mSendTick = HAL_GetTick();
	uart->writeStr(cmd, true);
	if (mEcho.tx) {
		switch (mEcho.timeMode) {
		case 0:
			getOutStream()->oMsgX(colMAGENTA, "%s", cmd);
			break;
		case 1:
		case 2:
			getOutStream()->oMsgX(colMAGENTA, "%u:%s", HAL_GetTick() - mStartTick, cmd);
			break;
		case 3: {
			char buf[20];
			TDATE tm;
			Rtc::ReadOnlyTime(&tm);
			TimeTools::TimeStrZZ(buf, &tm);
			getOutStream()->oMsgX(colMAGENTA, "%s:%s", buf, cmd);
		}
			break;
		}
	}
}

void Bg96Driver::writeCmdF(const char *pFormat, ...) {
	va_list ap;
	va_start(ap, pFormat);
	vsnprintf(mTmpBuf, sizeof(mTmpBuf), pFormat, ap);
	writeCmd(mTmpBuf);
	va_end(ap);
}

void Bg96Driver::showRxEcho(int idx, const char *txt) {
	char inpLine2[160];
	Token::chgCtrlChar(inpLine2, txt, sizeof(inpLine2));

	switch (mEcho.timeMode) {
	case 0:
		getOutStream()->oMsgX(colGREEN, "%u>%s", idx, inpLine2);
		break;
	case 3:
	case 1:
		getOutStream()->oMsgX(colGREEN, "%u>%u:%s", idx, HAL_GetTick() - mSendTick, inpLine2);
		break;
	case 2:
		getOutStream()->oMsgX(colGREEN, "%u>%u:%s", idx, HAL_GetTick() - mStartTick, inpLine2);
		break;
	}
}

void Bg96Driver::powerUp() {
	imAlive();
	this->mAlivePeriod = 30000; //30 sekund

	mPhase = phSTART;

	logT(vvHINT, "Właczenie zasilania BG96");
	set3_8V(false);
	osDelay(500);
	set3_8V(true);
	setReset(false);
	setDTR(false);
	osDelay(30);
	uart->clearInpStream();
	setPowerKey(true);
	osDelay(500);
	setPowerKey(false);

	uint32_t tm = HAL_GetTick();
	logT(vvHINT, "BG WaitForRdy");
	mPhase = phWAIT_FOR_STATUS;

	while (!asynch.doStop) {
		if (getHdStatus() == 0) {
			logT(vvHINT, "BG Ready, tm=%u[ms]", HAL_GetTick() - tm);
			break;
		}
		if (dwb(50) == stBREAK) {
			logT(vvHINT, "BREAK");
			return;
		}
	}
	osDelay(100);
	mPhase = phWAIT_STATUS_OK;
	uart->stat.rxCnt = 0;
	uart->stat.lnCnt = 0;
	imAlive();
	this->mAlivePeriod = 10000; //10 sekund
}

int Bg96Driver::cmpToArray(const char *txt, const char *const*array) {
	int k = 0;
	while (1) {
		const char *ptr = array[k];
		if (ptr == NULL)
			return stUNKN_REPL;
		if (strcmp(txt, ptr) == 0)
			return k;
		k++;
	}
}

int Bg96Driver::getAnswLn(char *repl, int max, int time) {
	int startTm = HAL_GetTick();

	while (true) {
		int t2 = time - (HAL_GetTick() - startTm);
		if (t2 <= 0)
			return stTIME_OUT;
		osEvent ev = osSignalWait(SIGNAL_CHAR | SIGNAL_BREAK, t2);
		if (ev.status == osEventSignal) {
			int code = ev.value.v;
			if (code & SIGNAL_BREAK) {
				return stBREAK;
			}
		}
		if (uart->readLn(repl, max)) {
			if (mEcho.rx) {
				showRxEcho(1, repl);
			}
			return stOK;
		}
	}
}

void Bg96Driver::parseCreg(const char *ptr) {
	int v1, v2;
	Token::getAsInt(&ptr, &v1);
	Token::getAsInt(&ptr, &v2);
	if (v1 == 2) {
		switch (v2) {
		case 0:
			recR.regStatus = regNO_REG;
			break;
		case 1:
			recR.regStatus = regREG_HOME;
			state.rdy.isCReg = true;
			break;
		case 2:
			recR.regStatus = regREG_UNKNOWN;
			break;
		case 3:
			recR.regStatus = regREG_DENIED;
			break;
		case 4:
			recR.regStatus = regREG_UNKNOWN;
			break;
		case 5:
			recR.regStatus = regREG_ROAMING;
			state.rdy.isCReg = true;
			break;
		default:
			recR.regStatus = regREG_UNKNOWN;
			break;
		}
	} else {
		recR.regStatus = regREG_UNKNOWN;
	}
}

bool Bg96Driver::parseTimeStr(const char *ptr, TDATE *tm) {
	int a1;
	Token::getAsInt(&ptr, "/,:", &a1);
	tm->rk = a1 - 2000;
	Token::getAsInt(&ptr, "/,:", &a1);
	tm->ms = a1;
	Token::getAsInt(&ptr, "/,:", &a1);
	tm->dz = a1;
	Token::getAsInt(&ptr, "/,:", &a1);
	tm->gd = a1;
	Token::getAsInt(&ptr, "/,:", &a1);
	tm->mn = a1;
	Token::getAsInt(&ptr, "/,:", &a1);
	tm->sc = a1;
	tm->se = 0; // nie ma informacji o setnych
	return true;
}
//+QNTP: 0,"2021/01/06,17:57:23+04"
void Bg96Driver::parseNtpTime(const char *ptr) {
	Token::getAsInt(&ptr, &state.ntpTime.recErrorCode);  //error code
	char buf[30];
	Token::get(&ptr, buf, sizeof(buf));
	Token::remooveQuota(buf);

	if (parseTimeStr(buf, &state.ntpTime.reciveTime)) {
		if (TimeTools::AddHour(&state.ntpTime.reciveTime, config->data.R.dev.timeZoneShift)) {
			state.ntpTime.reciveTime.timeSource = tmSrcNTP;
			Rtc::SetTime(&state.ntpTime.reciveTime);
			Rtc::SetDate(&state.ntpTime.reciveTime);
			config->data.R.dev.rtcSetUpTime = state.ntpTime.reciveTime;
			config->saveRtc();
		}
		state.ntpTime.recived = true;
		state.ntpTime.reciveTick = HAL_GetTick();
		state.ntpTime.tryCnt = 0;
	}
}

//czas z sieci GSM
//+CTZE: "+04",0,"2021/02/02,18:36:23"
void Bg96Driver::parseNetworkTime(const char *ptr) {

	char buf[30];
	Token::get(&ptr, buf, sizeof(buf));  //różnica do czasu GMT w kwadransach
	state.gsmTime.quatersDiff = atoi(buf);
	Token::getAsBool(&ptr, &state.gsmTime.isDayLightSaving);
	Token::get(&ptr, buf, sizeof(buf));
	if (parseTimeStr(buf, &state.gsmTime.reciveTime)) {
		int dHour = state.gsmTime.quatersDiff / 4;
		if (TimeTools::AddHour(&state.gsmTime.reciveTime, dHour)) {
			Rtc::SetTime(&state.gsmTime.reciveTime);
			Rtc::SetDate(&state.gsmTime.reciveTime);
			config->data.R.dev.rtcSetUpTime = state.gsmTime.reciveTime;
			config->saveRtc();
		}
		state.gsmTime.recived = true;
	}
}

//+QGPSLOC: 140458.0,50.17105,18.64469,1.3,210.0,2,186.20,0.0,0.0,020121,05
//+QGPSLOC: 175827.0,50.17100,18.64478,1.5,254.0,2,135.59,0.0,0.0,020121,03

void Bg96Driver::parseGpsData(const char *ptr) {
	char token[20];

	Token::get(&ptr, token, sizeof(token)); //czas: 140458
	state.gps.reciveTime.gd = Token::parseInt(&token[0], 2);
	state.gps.reciveTime.mn = Token::parseInt(&token[2], 2);
	state.gps.reciveTime.sc = Token::parseInt(&token[4], 2);
	state.gps.reciveTime.se = 0;

	Token::getAsFloat(&ptr, &state.gps.latitude);
	Token::getAsFloat(&ptr, &state.gps.longitude);
	Token::getAsFloat(&ptr, &state.gps.hdop); //hdop - horizontal precision
	Token::getAsFloat(&ptr, &state.gps.altitude); //altitude - wysokość nap poz.morza
	Token::getAsInt(&ptr, &state.gps.fix); //fix - posiotion code
	Token::getAsFloat(&ptr, &state.gps.cog); //cog - kierunek ruchu - kąt
	Token::getAsFloat(&ptr, &state.gps.spkm); //spkm - prędkość w km/h
	Token::get(&ptr, token, sizeof(token)); //spkn - prędkość w węzłach
	Token::get(&ptr, token, sizeof(token)); //date - aktualna data : 020121
	state.gps.reciveTime.dz = Token::parseInt(&token[0], 2);
	state.gps.reciveTime.ms = Token::parseInt(&token[2], 2);
	state.gps.reciveTime.rk = Token::parseInt(&token[4], 2);
	Token::getAsInt(&ptr, &state.gps.satCnt); //nsat - ilość satelit
//Token::get(&ptr, token,sizeof(token)); //errcode- error code

	state.gps.reciveTick = HAL_GetTick();
	state.gps.gpsFix = true;
	if (config->data.R.bg96.gps.setRtcTime) {
		if (TimeTools::AddHour(&state.gps.reciveTime, config->data.R.dev.timeZoneShift)) {
			state.gps.reciveTime.timeSource = tmSrcGPS;
			Rtc::SetTime(&state.gps.reciveTime);
			Rtc::SetDate(&state.gps.reciveTime);
			config->data.R.dev.rtcSetUpTime = state.gps.reciveTime;
			config->saveRtc();
		}
	}
	state.gps.tryCnt = 0;
}

void Bg96Driver::parseRSSI(const char *ptr) {
	int rssi, ber;
	Token::getAsInt(&ptr, &rssi);
	Token::getAsInt(&ptr, &ber);
	if (rssi == 0)
		state.rssi.rssi = -113;
	else if (rssi == 1)
		state.rssi.rssi = -111;
	else if (rssi >= 2 && rssi <= 30)
		state.rssi.rssi = -109 + (2 * (rssi - 2));
	else if (rssi == 31)
		state.rssi.rssi = -51;
	else
		state.rssi.rssi = NAN;
	state.rssi.ber = ber;
	/*
	 RXQUAL_0 BER < 0,2 % Assumed value = 0,14 %
	 RXQUAL_1 0,2 % < BER < 0,4 % Assumed value = 0,28 %
	 RXQUAL_2 0,4 % < BER < 0,8 % Assumed value = 0,57 %
	 RXQUAL_3 0,8 % < BER < 1,6 % Assumed value = 1,13 %
	 RXQUAL_4 1,6 % < BER < 3,2 % Assumed value = 2,26 %
	 RXQUAL_5 3,2 % < BER < 6,4 % Assumed value = 4,53 %
	 RXQUAL_6 6,4 % < BER < 12,8 % Assumed value = 9,05 %
	 RXQUAL_7 12,8 % < BER Assumed value = 18,10 %

	 */
}
void Bg96Driver::resolveLine() {

	if (strbcmp(recR.recBuf, "\r\n")) {
		recR.flags |= nnEmptyLn;
	} else if (strbcmp(recR.recBuf, "OK\r\n")) {
		recR.flags |= nnOK;
	} else if (strbcmp(recR.recBuf, "RDY\r\n")) {
		recR.flags |= nnRDY;

	} else if (strbcmp(recR.recBuf, "ERROR\r\n")) {
		recR.flags |= nnError;
	} else {
		Token::remooveEOL(recR.recBuf);
		if (recR.recBuf[0] == '+') {
			const char *ptr;
			int tmp;

			if (strbcmp2(recR.recBuf, "+CME ERROR:", &ptr)) {
				recR.flags |= nnErrCME;
				Token::getAsInt(&ptr, &recR.codeErrCME);
			} else if (strbcmp2(recR.recBuf, "+CMS ERROR:", &ptr)) {
				recR.flags |= nnErrCMS;
				Token::getAsInt(&ptr, &recR.codeErrCMS);
			} else if (strbcmp2(recR.recBuf, "+QMTSTAT:", &ptr)) {
				//+QMTSTAT: <tcpconnectID>,<err_code>
				recR.flags |= nnQmtStat;
				Token::getAsInt(&ptr, &tmp);
				Token::getAsInt(&ptr, &tmp);  //errCode
				if (tmp > 0) {
					state.mqtt.svrOpened = false;
					state.mqtt.closeCode = tmp;
				}

			} else if (strbcmp2(recR.recBuf, "+QMTPUB:", &ptr)) {
				//+QMTPUB: 0,0,0 -> <tcpconnectID>,<msgID>,<result>
				recR.flags |= nnQmtPub;
				Token::getAsInt(&ptr, &tmp);
				Token::getAsInt(&ptr, &tmp);
				Token::getAsInt(&ptr, &recR.wResult);
				state.mqtt.sentAckTick = HAL_GetTick();
			} else if (strbcmp2(recR.recBuf, "+QMTOPEN:", &ptr)) {
				//+QMTOPEN: <tcpconnectID>,<result>
				recR.flags |= nnQmtOpen;
				Token::getAsInt(&ptr, &tmp);
				Token::getAsInt(&ptr, &recR.qmtErr);
			} else if (strbcmp2(recR.recBuf, "+QMTCONN:", &ptr)) {
				//+QMTCONN: <tcpconnectID>,<result>[,<ret_code>]
				recR.flags |= nnQmtConn;
				Token::getAsInt(&ptr, &tmp);
				Token::getAsInt(&ptr, &recR.qmtErr);
				Token::getAsInt(&ptr, &recR.qmtRepl);
			} else if (strbcmp2(recR.recBuf, "+QMTCLOSE:", &ptr)) {
				recR.flags |= nnQmtClose;
			} else if (strbcmp2(recR.recBuf, "+QNTP:", &ptr)) {
				recR.flags |= nnQNtp;
				parseNtpTime(ptr);
			} else if (strbcmp2(recR.recBuf, "+CREG:", &ptr)) {
				recR.flags |= nnCreg;
				parseCreg(ptr);
			} else if (strbcmp2(recR.recBuf, "+QSSLCFG:", &ptr)) {
				recR.flags |= nnQSslCfg;
				Token::get(&ptr, recR.sslCfg.paramName, sizeof(recR.sslCfg.paramName));
				Token::getAsInt(&ptr, &recR.sslCfg.contextID);
				Token::get(&ptr, recR.sslCfg.paramValue, sizeof(recR.sslCfg.paramValue));
			} else if (strbcmp2(recR.recBuf, "+CSQ:", &ptr)) {
				recR.flags |= nnCsq;
				parseRSSI(ptr);
			} else if (strbcmp2(recR.recBuf, "+QGPSLOC:", &ptr)) {
				recR.flags |= nnQGpsLoc;
				parseGpsData(ptr);
			} else if (strbcmp2(recR.recBuf, "+QIACT:", &ptr)) {
				//+QIACT: 1,1,1,"46.76.183.88"
				//+QIACT: <context_id><context_state>,<context_type>,<IP_address>
				recR.flags |= nnQiact;
				int id, a1;
				Token::getAsInt(&ptr, &id); //numer kontekstu
				Token::getAsInt(&ptr, &a1); //czy aktywny
				Token::getAsInt(&ptr, &a1); // IPv4 vs IPv6
				Token::get(&ptr, bgParam.myIp, sizeof(bgParam.myIp)); // IPv4 vs IPv6
				Token::remooveQuota(bgParam.myIp);
				state.rdy.iAct = true;

			} else if (strbcmp2(recR.recBuf, "+CPIN:", &ptr)) {
				recR.flags |= nnCpin;
				Token::get(&ptr, bgParam.simStatus, sizeof(bgParam.simStatus));
				state.rdy.simPinOk = (strbcmp(bgParam.simStatus, "READY"));

			} else if (strbcmp2(recR.recBuf, "+QSIMSTAT:", &ptr)) {
				recR.flags |= nnQSimStat;
				Token::getAsInt(&ptr, &tmp);
				state.rdy.simCardMsgEnabled = (tmp == 1);
				Token::getAsInt(&ptr, &tmp);
				state.rdy.simCardInserted = tmp;

			} else if (strbcmp2(recR.recBuf, "+CTZE:", &ptr)) {
				recR.flags |= nnCtze;
				parseNetworkTime(ptr);
			} else if (strbcmp2(recR.recBuf, "+QFOPEN:", &ptr)) {
				recR.flags |= nnFOpen;
				Token::getAsInt(&ptr, &recR.fileNr);
			} else if (strbcmp2(recR.recBuf, "+CMTI:", &ptr)) {
				//+CMTI: "ME",0\r\n
				recR.flags |= nnCMTI;
				Token::get(&ptr);
				Token::getAsInt(&ptr, &recR.smsIdx);
			} else if (strbcmp2(recR.recBuf, "+CMGR:", &ptr)) {
				//+CMGR: "REC READ","+48607260595",,"21/01/12,19:56:34+04"\r\n
				recR.flags |= nnCMGR;
				Token::get(&ptr);
				Token::get(&ptr, state.sms.nrTel, sizeof(state.sms.nrTel));
			}
		} else {
			recR.flags |= nnNoURC;
		}
	}
}

int Bg96Driver::getReplUntil(uint32_t inFlags, int time) {
	//wyzerowanie flag, które będą obserwowane
	recR.flags &= ~inFlags;

	int tt = HAL_GetTick();
	while (1) {
		int t2 = time - (HAL_GetTick() - tt);
		if (t2 < 0)
			return stTIME_OUT;

		int st = getAnswLn(recR.recBuf, sizeof(recR.recBuf), time);
		if (st == stBREAK)
			return st;
		resolveLine();

//sprawdzenie czy zapalona jedna z oczekiwanych flag
		if ((recR.flags & inFlags) != 0) {
			return stOK;
		}
	}
}

//odbiera wszystko przez zadany czas. W zasadzie Wait
int Bg96Driver::getReplUntil(int time) {
	return getReplUntil(0, time);
}

int Bg96Driver::getReplWithErr(uint32_t flag, int time) {
	flag |= (nnErrCME | nnErrCMS | nnError);

	int st = getReplUntil(flag, time);
	if (st < stOK)
		return st;
	if (recR.flags & nnError)
		return -stBgError;
	if (recR.flags & nnErrCME)
		return -(ERR_BASE_CME + recR.codeErrCME);
	if (recR.flags & nnErrCMS)
		return -(ERR_BASE_CMS + recR.codeErrCMS);
	return stOK;
}

int Bg96Driver::getReplOk(int time) {
	int st = getReplWithErr(nnOK, time);
	if (st < stOK)
		return st;
	if (recR.flags & nnOK)
		return stOK;
	return stSOFT_EXCP;
}

int Bg96Driver::getBgData(const char *cmd, char *buf, int max, int time) {
	writeCmd(cmd);
	int st = getReplUntil(nnNoURC, time);
	if (st != stOK)
		return st;
	strncpy(buf, recR.recBuf, max);
	Token::remooveEOL(buf);
	return getReplOk(time);
}

/*OpenMode:
 * 0 - create
 * 1 - create and cler
 * 2 - open for read
 *
 */
int Bg96Driver::openBGFile(const char *fileName, int openMode, int *fileHandle) {
	writeCmdF("at+qfopen=\"%s\",%d", fileName, openMode);
	int st = getReplWithErr(nnFOpen, 300);
	if (st < stOK)
		return st;
	if (!(recR.flags & nnFOpen))
		return stSOFT_EXCP;

	*fileHandle = recR.fileNr;
	st = getReplOk(300);
	return st;
}

int Bg96Driver::closeBGFile(int fileHandle) {
	writeCmdF("at+qfclose=%d", fileHandle);
	int st = getReplOk(300);
	return st;
}

const CpxDescr SslCfgDscr[] = { //
		{ ctype : cpxBYTE, ofs: offsetof(SSL_Cfg, sslversion), Name : "sslversion", sizeof(SSL_Cfg::sslversion) }, //
				{ ctype : cpxBYTE, ofs: offsetof(SSL_Cfg, seclevel), Name : "seclevel", sizeof(SSL_Cfg::seclevel) }, //
				{ ctype : cpxHEXWORD, ofs: offsetof(SSL_Cfg, ciphersuite), Name : "ciphersuite", sizeof(SSL_Cfg::ciphersuite) }, //
				{ ctype : cpxQUOTASTR, ofs: offsetof(SSL_Cfg, cacert), Name : "cacert", sizeof(SSL_Cfg::cacert) }, //
				{ ctype : cpxQUOTASTR, ofs: offsetof(SSL_Cfg, clientcert), Name : "clientcert", sizeof(SSL_Cfg::clientcert) }, //
				{ ctype : cpxQUOTASTR, ofs: offsetof(SSL_Cfg, clientkey), Name : "clientkey", sizeof(SSL_Cfg::clientkey) }, //
				{ ctype : cpxBOOL, ofs: offsetof(SSL_Cfg, sni), Name : "sni", sizeof(SSL_Cfg::sni) }, //
				{ ctype : cpxBOOL, ofs: offsetof(SSL_Cfg, checkhost), Name : "checkhost", sizeof(SSL_Cfg::checkhost) }, //
				{ ctype : cpxBOOL, ofs: offsetof(SSL_Cfg, ignorecertchain), Name : "ignorecertchain", sizeof(SSL_Cfg::ignorecertchain) }, //
				{ ctype : cpxBOOL, ofs: offsetof(SSL_Cfg, ignorelocaltime), Name : "ignorelocaltime", sizeof(SSL_Cfg::ignorelocaltime) }, //
				{ ctype : cpxINT, ofs: offsetof(SSL_Cfg, negotiatetime), Name : "negotiatetime", sizeof(SSL_Cfg::negotiatetime) }, //
				{ ctype : cpxBOOL, ofs: offsetof(SSL_Cfg, dtls), Name : "dtls", sizeof(SSL_Cfg::dtls) }, //
				{ ctype : cpxINT, ofs: offsetof(SSL_Cfg, dtlsversion), Name : "dtlsversion", sizeof(SSL_Cfg::dtlsversion) }, //

				{ ctype : cpxNULL }

		};

int Bg96Driver::showSslContext(int contextNr) {
	SSL_Cfg sslCfg;
	memset(&sslCfg, 0, sizeof(sslCfg));

	const uint32_t FLAGS = nnQSslCfg | nnOK | nnError;

	Cpx cpx;
	cpx.init(SslCfgDscr, &sslCfg);

	int st = stOK;
	while (!cpx.isEof()) {
		writeCmdF("at+qsslcfg=\"%s\",%d", cpx.getDscrName(), contextNr);
		st = getReplUntil(FLAGS, 300);
		if (st < 0)
			break;
		if (recR.flags & nnError) {
			st = stBgError;
			break;
		}
		if (recR.flags & nnQSslCfg) {
			st = getReplOk(300);
			if (st < 0)
				break;
		}

		cpx.setItem(recR.sslCfg.paramValue);
		cpx.next();
	}
	if (st == stOK) {
		Cpx cpx;
		cpx.init(SslCfgDscr, &sslCfg);
		cpx.list(getOutStream());
	} else {
		getOutStream()->oMsgX(colRED, "Błąd odczytu, st=%d", st);
	}
	return stOK;
}

int Bg96Driver::setSslContext(int contextNr, const SSL_Cfg *pCfg) {

	Cpx cpx;
	cpx.init(SslCfgDscr, pCfg);

	while (!cpx.isEof()) {
		char buf[40];
		cpx.getAsTxt(buf, sizeof(buf));
		if (buf[0] != 0) {
			writeCmdF("at+qsslcfg=\"%s\",%d,%s", cpx.getDscrName(), contextNr, buf);
			int st = getReplOk(300);
			if (st < 0) {
				return st;
			}
		}
		cpx.next();
	}

	cpx.init(SslCfgDscr, pCfg);
	cpx.list(getOutStream());
	return stOK;
}

#define MQTT_SSL_CONTEXT_IDX  0
#define MQTT_IDX  1

int Bg96Driver::openMqqtConnection() {
	uint32_t tt = HAL_GetTick();
	if (state.mqtt.tryOpenTick != 0 && (tt - state.mqtt.tryOpenTick < 4000)) {
		return stNOT_LOGED;
	}
	state.mqtt.tryOpenTick = tt;

	//Próba co 4 sekundy

	//doFlushInpBuffer();

//ustawienie SSL
	writeCmdF("at+qmtcfg=\"ssl\",%d,%d,%d", MQTT_IDX, config->data.R.mqtt.useSSL, MQTT_SSL_CONTEXT_IDX);
	int st = getReplOk(300);
	if (st < 0) {
		state.mqtt.errorOpenCnt++;
		return st;
	}
//ustawienie KeepAlive
	if (config->data.R.mqtt.keepAlive > 0) {
		writeCmdF("at+qmtcfg=\"keepalive\",%d,%d", MQTT_IDX, config->data.R.mqtt.keepAlive);
		st = getReplOk(300);
		if (st < 0) {
			state.mqtt.errorOpenCnt++;
			return st;
		}
	}

//Otwarcie połączenia
	word port = config->data.R.mqtt.SvrPortNoSSL;
	if (config->data.R.mqtt.useSSL) {
		port = config->data.R.mqtt.SvrPortSSL;
	}
	if (config->data.R.mqtt.usePSK) {
		port = config->data.R.mqtt.SvrPortPSK;
	}
	writeCmdF("at+qmtopen=%d,\"%s\",%d", MQTT_IDX, config->data.R.mqtt.SvrName, port);
	st = getReplOk(300);
	if (st < 0) {
		state.mqtt.errorOpenCnt++;
		return st;
	}
	st = getReplWithErr(nnQmtOpen, 1000 * config->data.R.mqtt.maxLoginTime);
	if (st < 0) {
		state.mqtt.errorOpenCnt++;
		return st;
	}

	if (recR.qmtErr != 0) {
		if (recR.qmtErr == 2) {
			//MQTT identifier is occupied
			state.mqtt.errorOpenCnt++;
		}

		if (recR.qmtErr == 3) {
			//Failed to activate PDP -> restart
			state.mqtt.errorOpenCnt++;
		}
		return -(ERR_BASE_QMTOPEN + recR.qmtErr);
	}

//Logowanie urn:im40:123456781234567
	writeCmdF("at+qmtconn=%d,\"urn:im40:%s\",\"%s\",\"%s\"", MQTT_IDX, bgParam.imei, config->data.R.mqtt.userName, config->data.R.mqtt.password);
	//writeCmdF("at+qmtconn=%d,\"plytka:1\",\"%s\",\"%s\"", MQTT_IDX, config->data.R.mqtt.userName, config->data.R.mqtt.password);
	st = getReplOk(300);
	if (st < 0) {
		state.mqtt.errorOpenCnt++;
		return st;
	}
	st = getReplWithErr(nnQmtConn, 1000 * config->data.R.mqtt.maxLoginTime);
	if (st < 0) {
		state.mqtt.errorOpenCnt++;
		return st;
	}
	if (recR.qmtErr != 0) {
		state.mqtt.errorOpenCnt++;
		return -(ERR_BASE_QMTCONN + recR.qmtRepl);
	}

	state.mqtt.svrOpened = true;
	state.mqtt.mSendMsgID = 1;
	state.mqtt.errorOpenCnt = 0;

	return stOK;
}

int Bg96Driver::closeMqqtConnection() {
	writeCmdF("at+qmtclose=%d", MQTT_IDX);
	state.mqtt.svrOpened = false;
	int st = getReplOk(300);
	return st;
}

int Bg96Driver::sendMqqtData(const char *varName, const char *data, bool doDataCpy) {
	int msgID = 0;
	if (config->data.R.mqtt.qos > 0)
		msgID = state.mqtt.mSendMsgID++;

	writeCmdF("at+qmtpub=%d,%d,%d,%d,\"%s\"", MQTT_IDX, msgID, config->data.R.mqtt.qos, config->data.R.mqtt.retain, varName);

	int st = getReplUntil(nnEmptyLn | nnNoURC, 300);
	if (st != stOK)
		return st;

	writeCmdNoEcho(data, doDataCpy);
	if (mEcho.tx) {
		int n = strlen(data);
		getOutStream()->oMsgX(colMAGENTA, "%u:*Dane* :Len=%u", HAL_GetTick() - mSendTick, n);
	}

	st = getReplWithErr(nnQmtPub, 20000); //todo: dorobić zależności od pkt_timeout i retry_times
	if (st != stOK)
		return st;
	if (recR.wResult != 0)
		return -(ERR_BASE_QMTPUB + recR.wResult);
	logT(vvTEXT, "MQTT sent");

	return st;
}

int Bg96Driver::sendMqqt(int idx, int val) {
	const char *dt = NULL;
	const char *var = NULL;
	char buf[20];

	switch (idx) {
	default:
	case 0:
		if (GlobData::buildExportJson() > 0) {
			dt = GlobData::jsonbuf->p();
			var = config->data.R.mqtt.varNamePub;
		}
		break;
	case 1:
		snprintf(buf, sizeof(buf), "%u" CTRL_Z, val);
		dt = buf;
		var = config->data.R.mqtt.varNamePub2;
		break;
	}
	if (var != NULL) {
		int st = sendMqqtData(var, dt, false);
		if (idx == 0) {
			if (st == stOK) {
				state.mqtt.sentTick = HAL_GetTick();
			}
		}
		return st;
	} else
		return stNO_DATA;
}

int Bg96Driver::turnOnGps() {
	writeCmd("at+qgps=1");

	int st = getReplOk(300);
	if (st < 0) {
		int err = -st;
		if (err > ERR_BASE_CME) {
			err -= ERR_BASE_CME;
			state.gps.errorCode = err;
		}
		return st;
	}
	return st;
}

int Bg96Driver::reciveNtpTime() {
	state.ntpTime.tryTick = HAL_GetTick();
	state.ntpTime.tryCnt++;
	writeCmdF("at+qntp=1,\"%s\",123", config->data.R.bg96.ntp.SvrName);
	int st = getReplOk(300);
	return st;
}

int Bg96Driver::reciveRSSI() {
	state.rssi.sndReqTick = HAL_GetTick();
	state.rssi.recivedOK = false;
	state.rssi.tryCnt++;

	writeCmd("at+csq");
	int st = getReplUntil(nnCsq, 300);
	if (st < 0)
		return st;
	st = getReplOk(300);
	if (st < 0)
		return st;
	state.rssi.recivedOK = true;
	return st;
}

int Bg96Driver::readGpsData() {
	state.gps.tryTick = HAL_GetTick();
	state.gps.tryCnt++;

	writeCmd("at+qgpsloc=2");
	int st = getReplWithErr(nnQGpsLoc, 500);
	if (st == stOK) {
		st = getReplOk(300);
	} else if (st - ERR_BASE_CME == 505) {
		logT(vvTEXT, "włączenie GPS");
		writeCmd("at+qgps=1");
		st = getReplOk(300);
	}
	return st;
}

int Bg96Driver::SendPIN() {
	// odczytanie statusu czy karta jest wożona
	writeCmd("at+qsimstat?");
	int st = getReplWithErr(nnQSimStat, 300);
	if (st < 0)
		return st;
	if (state.rdy.simCardInserted != 1)  // jeśli karta SIM nie jest włożona
		return stNO_SIM_CARD;

	if (strlen(config->data.R.bg96.SimPin) > 0) {
		writeCmdF("at+cpin=\"%s\"", config->data.R.bg96.SimPin);
		st = getReplOk(5000);
		if (st < 0)
			return st;
		for (int i = 0; i < 10; i++) {
			st = dwb(400);
			if (st == stBREAK)
				break;
			writeCmd("at+cpin?");
			st = getReplWithErr(nnCpin, 5000);
			if (st == stOK) {
				st = getReplOk(300);
				if (st < 0)
					return st;
				if (state.rdy.simPinOk)
					break;
				else
					st = stNOT_LOGED;
			}
		}
	}
	if (st == stOK) {
		st = getBgData("at+cimi", bgParam.sim_imsi, sizeof(bgParam.sim_imsi), 300);
	}

	return stOK;
}

RegStatus Bg96Driver::checkNetworkRegisterStatus() {
	recR.regStatus = regREG_UNKNOWN;
	writeCmd("at+creg?");
	int st = getReplUntil(nnCreg, 5000);
	if (st == stOK)
		st = getReplOk(300);
	return recR.regStatus;
}

int Bg96Driver::registerToNetwork() {
// sprawdzenie czy już zalogowany
	RegStatus reg = checkNetworkRegisterStatus();
	if (reg == regREG_HOME || reg == regREG_ROAMING) {
		bgParam.mRegStatus = reg;
		return stOK;
	}

//logowanie do sieci
	writeCmd("at+creg=2");
	int st = getReplOk(300);
	if (st < 0)
		return st;

	for (int i = 0; i < 25; i++) {
		imAlive();
		st = dwb(5000);
		if (st == stBREAK)
			break;

		reg = checkNetworkRegisterStatus();

		switch (reg) {
		case regREG_HOME:
		case regREG_ROAMING:
			bgParam.mRegStatus = reg;
			return st;

		case regERROR:
		case regREG_DENIED:
		case regREG_UNKNOWN:
		case regREG_BADREPLAY:
			st = stREG_ERROR;
			bgParam.mRegStatus = reg;
			break;

		case regNO_REG:
			break;
		}
	}
	return stREG_ERROR;
}

int Bg96Driver::checkAPNrdy() {
	int st = stNOT_LOGED;
	for (int rep = 0; rep < 3; rep++) {
		writeCmd("at+qiact?");
		bool Logged = false;
		while (1) {
			st = getReplUntil(nnOK | nnQiact, 3000);
			if (st < 0)
				return st;
			if (recR.flags & nnQiact) {
				Logged = true;
			} else {
				break;
			}
		}
		if (!Logged)
			st = stNOT_LOGED;
		else
			break;
	}
	return st;
}

int Bg96Driver::enterAPN() {
	int st = checkAPNrdy();
	if (st == stOK)
		return stOK;

//logowanie do serwera APN
	writeCmdF("at+qicsgp=1,1,\"%s\",\"\",\"\",0", config->data.R.bg96.ApnName);
	st = getReplOk(300);
	if (st < 0)
		return st;
	writeCmd("at+qiact=1");
	st = getReplOk(60 * 1000); //do 60 sekund
	if (st < 0)
		return st;
	osDelay(1000);
	return checkAPNrdy();
}

int Bg96Driver::initBg() {
	imAlive();
	this->mAlivePeriod = 120000; //2 minuty na inicjalizacje

	mStartTick = HAL_GetTick();
	zeroState();

	mPhase = phFIRST_AT;
	writeCmd("at");
	int st = getReplUntil(nnOK | nnRDY, 300);
	if (st < 0)
		return st;
	mPhase = phFIRST_AT_OK;
	getReplUntil(4000);

	writeCmd("ate0");
	st = getReplOk(3000);
	if (st != stOK)
		return st;
	getReplUntil(800);

	writeCmd("at");
	st = getReplOk(1000);

	getReplUntil(400);

	//uart->clearInpStream();

	st = getBgData("at+qgmr", bgParam.firmVer, sizeof(bgParam.firmVer), 300);
	if (st < 0)
		return st;
	st = getBgData("at+gsn", bgParam.imei, sizeof(bgParam.imei), 300);
	if (st < 0)
		return st;
	st = getBgData("at+gmm", bgParam.model, sizeof(bgParam.model), 300);
	if (st < 0)
		return st;
	mPhase = phPARAMS_OK;

	// przełączenie komunikatów URC na port "uart1"
	writeCmd("at+qurccfg=\"urcport\",\"uart1\"");
	st = getReplOk(1000);
	if (st < 0)
		return st;

	int fileHandle;
	st = openBGFile("0_server.psk", 2, &fileHandle);
	if (st == stOK) {
		bgParam.file_0_server_exist = true;
		closeBGFile(fileHandle);
	}

	st = SendPIN();
	if (st < 0)
		return st;
	mPhase = phPIN_OK;

	//Message format
	writeCmd("at+cmgf=1"); //tryb textowy
	st = getReplOk(1000);
	if (st < 0)
		return st;

	//skasowanie wszystkich SMS z pamięci
	writeCmd("at+cmgd=1,4");
	st = getReplOk(1000);
	if (st < 0)
		return st;

	st = registerToNetwork();
	if (st < 0)
		return st;
	mPhase = phCREG_OK;

	st = enterAPN();
	if (st < 0)
		return st;
	mPhase = phLOGGED_APN;

	if (strlen(config->data.R.bg96.ntp.SvrName) > 0) {
		state.ntpTime.configured = true;
		st = reciveNtpTime();
	}

// jeśli załączony GPS
	if (config->data.R.bg96.gps.Mode != gpsOFF) {
		st = turnOnGps();
		if (st < 0)
			return st;
	}

	return stOK;

}

void Bg96Driver::doFlushInpBuffer() {
	int n = 0;
	while (n < 20) {
		bool q = uart->readLn(recR.recBuf, sizeof(recR.recBuf));
		if (!q)
			break;
		if (mEcho.rx) {
			showRxEcho(2, recR.recBuf);
		}
		resolveLine();
		n++;
	}
}

void Bg96Driver::rssiLoopFun() {
	if (state.rdy.isCReg) {
		bool q = false;

		if (state.rssi.sndReqTick == 0) {
			//nigdy jeszcze nie było czytane
			q = true;
			state.rssi.tryCnt = 0;
		}

		if (!q) {
			// ponowienie próby odczytu
			if (!state.rssi.recivedOK && state.rssi.tryCnt < 3) {
				if (HAL_GetTick() - state.rssi.recivedTick > 5000) {
					state.rssi.tryCnt++;
					q = true;
				}
			}
		}

		if (!q) {
			// odczyt co skonfigurowany czas
			if (HAL_GetTick() - state.rssi.recivedTick > (uint32_t) (1000 * config->data.R.bg96.rssiRefreshTime)) {
				q = true;
				state.rssi.tryCnt = 0;
			}
		}

		if (q) {
			if (reciveRSSI() == HAL_OK) {
				state.rssi.recivedTick = HAL_GetTick();
			}
		}
	}
}

void Bg96Driver::gpsLoopFun() {
	bool doRd = false;
	uint32_t dt = HAL_GetTick() - state.gps.tryTick;

	switch (config->data.R.bg96.gps.Mode) {
	case gpsONE_TIME:
		if (!state.gps.gpsFix) {
			if (state.gps.tryCnt < GPS_ONE_TRY_MAX) {
				if (dt > GPS_ONE_TRY_TIME) {  //co 10 sekund
					doRd = true;
				}
			}
		}
		if (asynch.doGetGps) {
			asynch.doGetGps = false;
			doRd = true;
		}
		break;
	case gpsRUN:
		if (config->data.R.bg96.gps.refreshTime >= 5) {
			if (dt > (uint32_t) (1000 * config->data.R.bg96.gps.refreshTime)) {
				state.gps.tryCnt = 0;
				doRd = true;
			}
			if (state.gps.tryCnt > 0 && state.gps.tryCnt < GPS_TRY_MAX) {
				if (dt > GPS_TRY_TIME) {  //co 2 sekund
					doRd = true;
				}
			}
			if (asynch.doGetGps) {
				asynch.doGetGps = false;
				doRd = true;
			}
		}
		break;
	}

	if (doRd) {
		readGpsData();
		if (config->data.R.bg96.gps.Mode == gpsONE_TIME && state.gps.gpsFix) {
			//wyłączenie obsługi GPS
			logT(vvTEXT, "GPS recived. Turn OFF");
			writeCmd("at+qgpsend");
			getReplOk(300);
		}
	}
}

void Bg96Driver::ntpLoopFun() {
	if (state.rdy.iAct) {
		if ((strlen(config->data.R.bg96.ntp.SvrName) > 0) && (config->data.R.bg96.ntp.RefreshTime > 1)) {
			bool doRd = false;
			if (state.ntpTime.tryCnt == 0)
				doRd = (HAL_GetTick() - state.ntpTime.tryTick > (uint32_t) (60000 * config->data.R.bg96.ntp.RefreshTime));
			else
				doRd = (HAL_GetTick() - state.ntpTime.tryTick > 10000); //10sekund
			if (asynch.doGetNtpTime) {
				asynch.doGetNtpTime = false;
				doRd = true;
			}
			if (doRd) {
				reciveNtpTime();
			}
		}
		if (recR.flags & nnQNtp) {
			recR.flags &= ~nnQNtp;
		}
	}
}

void Bg96Driver::mqttLoopFun() {

	if (strlen(config->data.R.mqtt.varNamePub) == 0)
		return;

	if (config->data.R.mqtt.mqttSendInterval <= 0)
		return;

	if (!state.rdy.iAct) {
		return;
	}

	int st = stOK;

	if (!state.mqtt.svrOpened) {
		if (config->data.R.mqtt.autoOpenMqttSvr) {
			st = openMqqtConnection();
		}
	}

	if (!state.mqtt.svrOpened) {
		return;
	}

	if ((state.mqtt.sentTick == 0) || (HAL_GetTick() - state.mqtt.sentTick > (uint32_t) (1000 * config->data.R.mqtt.mqttSendInterval))) {

		if (GlobData::buildExportJson() >= 0) {

			state.mqtt.trySendTick = HAL_GetTick();
			st = sendMqqtData(config->data.R.mqtt.varNamePub, GlobData::jsonbuf->p(),
			false);
			if (st == stOK) {
				state.mqtt.sentTick = HAL_GetTick();
				state.mqtt.errorSendCnt = 0;
			} else {
				closeMqqtConnection();
				state.mqtt.errorSendCnt++;
			}
		}
	}
}

void Bg96Driver::execNewSMS() {
	getOutStream()->oMsgX(colBLUE, "SMS [%s]: %s", state.sms.nrTel, state.sms.msg);
	const char *ptr = state.sms.msg;
	char tok[40];
	bool repl = false;

	Token::get(&ptr, tok, sizeof(tok));
	if (strcmp(tok, bgParam.imei) == 0) {
		getOutStream()->oMsgX(colBLUE, "SMS: IMEI OK");
		Token::get(&ptr, tok, sizeof(tok));
		if (strcmp(tok, "ZERO-GAS") == 0) {
			getOutStream()->oMsgX(colBLUE, "SMS: ZERO-GAS");
			if (airDetRs != NULL)
				airDetRs->zeroGasFromSMS(ptr, state.sendSms.msg, sizeof(state.sendSms.msg) - 1);
			repl = true;
		} else if (strcmp(tok, "REBOOT") == 0) {
			getOutStream()->oMsgX(colBLUE, "SMS: REBOOT");
			strlcpy(state.sendSms.msg, "REBOOTING", sizeof(state.sendSms.msg));
			reboot(2000);
			repl = true;
		}
	}
	if (repl) {
		int n = strlen(state.sendSms.msg);
		state.sendSms.msg[n] = 26;
		state.sendSms.msg[n + 1] = 0;
		strlcpy(state.sendSms.nrTel, state.sms.nrTel, sizeof(state.sendSms.nrTel));
		state.sendSms.flagSend = true;
	}
}

void Bg96Driver::smsLoopFun() {
	if (recR.flags & nnCMTI) {
		recR.flags &= ~nnCMTI;
		writeCmdF("at+cmgr=%d", recR.smsIdx);
		int st = getReplUntil(nnCMGR | nnOK, 100);
		if (st < 0)
			return;
		if (recR.flags & nnCMGR) {
			st = getAnswLn(state.sms.msg, sizeof(state.sms.msg), 100);
			if (st < 0)
				return;
			Token::remooveEOL(state.sms.msg);
			state.sms.reciveTick = HAL_GetTick();
			st = getReplOk(100);
			if (st < 0)
				return;
			writeCmd("at+cmgd=1,4");  //usunięcie wszustkich wiadomości
			st = getReplOk(100);
			execNewSMS();
		}
	}
}

//AT+CMGS="+48607260595"

void Bg96Driver::smsSendLoopFun() {
	if (state.sendSms.flagSend) {
		state.sendSms.flagSend = false;
		writeCmdF("at+cmgs=\"%s\"", state.sendSms.nrTel);
		int st = getReplUntil(nnEmptyLn, 1000);
		if (st == stOK) {
			writeCmd(state.sendSms.msg);
		}
	}
}

bool Bg96Driver::isMqttSendingOk() {
	if (state.mqtt.sentTick == 0)
		return false;
	return (HAL_GetTick() - state.mqtt.sentTick < (uint32_t) (1000 * (20 + config->data.R.mqtt.mqttSendInterval)));
}

// czy ma byś cykliczne wysyłanie pakietów MQTT
bool Bg96Driver::isMqttAutoRun() {
	return (config->data.R.mqtt.autoOpenMqttSvr) && (config->data.R.mqtt.mqttSendInterval >= 0) && (strlen(config->data.R.mqtt.varNamePub) > 0);
}

SSL_Cfg mySslCfg = { //
		sslversion:4,  //
				seclevel:0,  //
				ciphersuite:0xffff,  //

				cacert: { 'a', '1', '.', 't', 'x', 't', 0 },   //
				clientcert: { },  //
				clientkey: { },  //
				sni:false, //
				checkhost:false, //
				ignorecertchain:true, //
				ignorelocaltime:true, //
				negotiatetime:300, //
				dtls:true, //
				dtlsversion:2,

		};

void Bg96Driver::runLoop() {
	asynch.doStop = false;

	while (asynch.mRunning) {

		powerUp();
		if (asynch.doStop)
			return;

		int st = initBg();
		this->mAlivePeriod = 10000; //10 sekund

		if (st == stOK) {
			logT(vvHINT, "BG InitOK");
		} else {
			logT(vvERROR, "BG InitError st=%d", st);
		}

		state.mqtt.sentAckTick = HAL_GetTick(); // odliczany czas ma być względny

		if ((mPhase >= phCREG_OK)) {

			if (asynch.doStop)
				return;

			logT(vvTEXT, "BG RunLoop");

			if (config->data.R.mqtt.autoOpenMqttSvr)
				asynch.doOpenMqtt = true;

			while (asynch.mRunning) {
				imAlive();
				if (asynch.doStop) {
					asynch.mRunning = false;
				}

				doFlushInpBuffer();
				rssiLoopFun();
				doFlushInpBuffer();

				if (asynch.doOpenMqtt) {
					asynch.doOpenMqtt = false;
					int st = openMqqtConnection();
					logT(vvHINT, "MQTT Open st=%d", st);
				}

				if (asynch.doCloseMqtt) {
					asynch.doCloseMqtt = false;
					int st = closeMqqtConnection();
					logT(vvHINT, "MQTT Close st=%d", st);
				}
				if (asynch.doSendMqtt) {
					asynch.doSendMqtt = false;
					int st = sendMqqt(asynch.sendMqqtIdx, asynch.sendMqqtVal);
					logT(vvHINT, "MQTT Send st=%d", st);
				}

				if (asynch.doShowSSlContext) {
					asynch.doShowSSlContext = 0;
					showSslContext(asynch.sslContext);
				}
				if (asynch.doSetSSlContext) {
					asynch.doSetSSlContext = 0;
					setSslContext(asynch.sslContext, &mySslCfg);

				}

				doFlushInpBuffer();
				gpsLoopFun();
				ntpLoopFun();
				mqttLoopFun();
				smsLoopFun();
				smsSendLoopFun();

				osSignalWait(SIGNAL_CHAR, 100);
				doFlushInpBuffer();

				loopCnt++;
				if ((state.mqtt.errorSendCnt > 5) || (state.mqtt.errorOpenCnt > 2)) {
					logT(vvERROR, "BG96 Too much errors");
					break;
				}
				if (asynch.doRestart) {
					asynch.doRestart = false;
					logT(vvHINT, "BG96 Restart");
					break;
				}
				if (isMqttAutoRun()) {
					dword tt = HAL_GetTick() - state.mqtt.sentAckTick;
					dword tz = (config->data.R.mqtt.mqttSendInterval + 180) * 1000;
					if (tt > tz) {
						logT(vvERROR, "MQTT nie wysłane w zadanym czasie. Restart BG96.");
						break;
					}
				}

			}
		} else {
			//jeśli nie udało się zalogować do sieci to restart
			osDelay(5000);
		}

		if (uart->stat.rxCnt < 10) {
			logT(vvHINT, "Brak danych z UART'a. UartInit.");
			uart->Init();
			state.uart.errorCnt++;
			state.uart.uartRestartCnt++;
			if (state.uart.errorCnt >= 3) {

				logT(vvERROR, "UART Error. Reboot");
				reboot(1000);
			}

		} else {
			state.uart.errorCnt = 0;
		}

//wyłaczanie zasilania modemu
		logT(vvHINT, "Wyłaczenie zasilania BG96");
		set3_8V(false);
		osDelay(2000);
	}
}

void Bg96Driver::idleLoop() {
	asynch.doStop = false;
	asynch.doPowerUp = false;
	mStartTick = HAL_GetTick();

	while (!asynch.mRunning) {
		imAlive();
		if (asynch.doStart) {
			asynch.mRunning = true;
		}
		if (asynch.doPowerUp) {
			asynch.doPowerUp = false;
			powerUp();
		}
		osEvent ev = osSignalWait(SIGNAL_CHAR | SIGNAL_BREAK, 100);
		if (ev.status == osEventSignal) {
			int code = ev.value.v;
			if (code & SIGNAL_CHAR) {
				char inpLine[80];
				if (uart->readLn(inpLine, sizeof(inpLine))) {
					showRxEcho(2, inpLine);
				}
			}
		}
		loopCnt++;

	}
}

void Bg96Driver::ThreadFunc() {
	asynch.mRunning = false;
	asynch.doStart = false;
	asynch.doPowerUp = false;

	uart->setThreadId(getThreadId());
	uart->Init();

	loopCnt = 0;

	logT(vvHINT, "Bg96Driver START");

	if (config->data.R.bg96.autoStart)
		asynch.mRunning = true;

	while (1) {
		if (!asynch.mRunning) {
			mPhase = phSTOP;
			idleLoop();
		} else {
			mPhase = phSTART;
			runLoop();
			asynch.doStart = false;
			asynch.doPowerUp = false;
		}
	}
}

extern "C" const char* getPhaseName(BgPhase ph) {
	switch (ph) {
	case phSTOP:
		return "STOP";
	case phSTART:
		return "START";
	case phWAIT_FOR_STATUS:
		return "WAIT_FOR_STATUS";
	case phWAIT_STATUS_OK:
		return "WAIT_STATUS_OK";
	case phFIRST_AT:
		return "FIRST_AT";
	case phFIRST_AT_OK:
		return "FIRST_AT_OK";
	case phPARAMS_OK:
		return "PARAMS_OK";
	case phPIN_OK:
		return "PIN_OK";
	case phCREG_OK:
		return "CREG_OK";
	case phLOGGED_APN:
		return "LOGGED_APN";
	default:
		return "???";
	}
}

bool Bg96Driver::setEchoMode(const char *cmd) {
	if (strlen(cmd) >= 3) {
		mEcho.rx = (cmd[0] == '1');
		mEcho.tx = (cmd[1] == '1');
		mEcho.timeMode = cmd[2] - '0';
		mEcho.logV = cmd[3] - '0';
		if (mEcho.timeMode < 0 || mEcho.timeMode > 3)
			mEcho.timeMode = 0;
		if (mEcho.logV < 0 || mEcho.logV > vvTEXT)
			mEcho.logV = 0;

		return true;
	} else {
		return false;
	}
}

//-------------------------------------------------------------------------------------------------------------
void Bg96Driver::showState(OutStream *strm) {
	if (strm->oOpen(colWHITE)) {
		strm->oMsg("Running           : %s", YN(asynch.mRunning));
		strm->oMsg("Phase             : %u - %s", mPhase, getPhaseName(mPhase));
		strm->oMsg("Reg.status        : %s", getRegStatusAsName(bgParam.mRegStatus));
		if (!isnanf(state.rssi.rssi)) {
			strm->oMsg("RSSI              : %.1f[dB] (%.3f[s])", state.rssi.rssi, 0.001 * (HAL_GetTick() - state.rssi.recivedTick));
		} else {
			strm->oMsg("RSSI              : brak");
		}
		strm->oMsg("BER               : %.1f", state.rssi.ber);

		strm->oMsg("LoopCnt           : %u", loopCnt);
		strm->oMsg("uart.rxCnt        : %u", uart->stat.rxCnt);
		strm->oMsg("uart.lnCnt        : %u", uart->stat.lnCnt);
		strm->oMsg("uart.toBigLineCnt : %u", uart->stat.toBigLineCnt);
		strm->oMsg("uart.errorCnt     : %u", state.uart.errorCnt);
		strm->oMsg("uart.restartCnt   : %u", state.uart.uartRestartCnt);
		strm->oMsg("echo              : rx=%u tx=%u timeMode=%u logLevel=%u", mEcho.rx, mEcho.tx, mEcho.timeMode, mEcho.logV);

		strm->oClose();
	}

}
void Bg96Driver::showHdwState(OutStream *strm) {
	if (strm->oOpen(colWHITE)) {
		strm->oMsg("Out #Pwr3_8V: %s", YN(get3_8V()));
		strm->oMsg("Out Reset   : %s", HL(getReset()));
		strm->oMsg("Out PowerKey: %s", HL(getPowerKey()));
		strm->oMsg("Out DTR     : %s", HL(getDTR()));
		strm->oMsg("In  ApReady : %s", HL(getApReady()));
		strm->oMsg("In  HdStatus: %s", HL(getHdStatus()));
		strm->oMsg("In  DCD     : %s", HL(getDCD()));
		strm->oMsg("In  RI      : %s", HL(getRI()));
		strm->oClose();
	}
}

void Bg96Driver::showInformation(OutStream *strm) {
	if (strm->oOpen(colWHITE)) {
		strm->oMsg("Model  : %s", bgParam.model);
		strm->oMsg("FirmVer: %s", bgParam.firmVer);
		strm->oMsg("IMEI   : %s", bgParam.imei);
		strm->oMsg("IMSI   : %s", bgParam.sim_imsi);

		strm->oMsg("File 0_server.psk : %s", YN(bgParam.file_0_server_exist));
		strm->oMsg("myIP   : %s", bgParam.myIp);
		if (state.gsmTime.recived) {
			char buf[30];
			TimeTools::DtTmStr(buf, &state.gsmTime.reciveTime);
			strm->oMsg("GSM_TM : %s", buf);
		} else
			strm->oMsg("GSM_TM : brak");

		if (state.ntpTime.configured) {
			if (state.ntpTime.recived) {
				char buf[30];
				TimeTools::DtTmStr(buf, &state.ntpTime.reciveTime);
				strm->oMsg("NTP    : %s", buf);
			} else
				strm->oMsg("NTP    : error %d", state.ntpTime.recErrorCode);

		} else
			strm->oMsg("NTP    : nie skonfigurowany");
		strm->oClose();
	}
}

void Bg96Driver::showGpsInformation(OutStream *strm) {
	if (strm->oOpen(colWHITE)) {
		strm->oMsg("GpsFix         :%s", YN(state.gps.gpsFix));
		if (state.gps.gpsFix) {
			strm->oMsg("Ilość sat.     :%d", state.gps.satCnt);
			strm->oMsg("Latitude (sz)  :%.5f", state.gps.latitude);
			strm->oMsg("Longitude (dł) :%.5f", state.gps.longitude);
			strm->oMsg("Precyzja       :%.1f[m]", state.gps.hdop); //  horizontal precision
			strm->oMsg("Wysokość       :%.2f[m]", state.gps.altitude); // wysokość nap poz.morza
			strm->oMsg("Kod pozycji    :%uD", state.gps.fix); // posiotion code
			strm->oMsg("Kierunek ruchu :%.2f[stop]", state.gps.cog); // kierunek ruchu - kąt
			strm->oMsg("Prędkość       :%.2f[km/h]", state.gps.spkm); // prędkość w km/h
			char buf[30];
			TimeTools::DtTmStr(buf, &state.gps.reciveTime);
			strm->oMsg("Odebrany czas  :%s", buf);
		}
		strm->oClose();
	}
}

void Bg96Driver::funShowState(OutStream *strm, const char *cmd, void *arg) {
	Bg96Driver *dev = (Bg96Driver*) arg;
	dev->showState(strm);
}

void Bg96Driver::funShowHdwState(OutStream *strm, const char *cmd, void *arg) {
	Bg96Driver *dev = (Bg96Driver*) arg;
	dev->showHdwState(strm);
}
void Bg96Driver::funInfo(OutStream *strm, const char *cmd, void *arg) {
	Bg96Driver *dev = (Bg96Driver*) arg;
	dev->showInformation(strm);
}
void Bg96Driver::funGpsInfo(OutStream *strm, const char *cmd, void *arg) {
	Bg96Driver *dev = (Bg96Driver*) arg;
	dev->showGpsInformation(strm);
}

void Bg96Driver::funOnOff3_8(OutStream *strm, const char *cmd, void *arg) {
	Bg96Driver *dev = (Bg96Driver*) arg;
	bool q;
	Token::getAsBool(&cmd, &q);
	dev->set3_8V(q);
}

void Bg96Driver::funSetResetLine(OutStream *strm, const char *cmd, void *arg) {
	Bg96Driver *dev = (Bg96Driver*) arg;
	bool q;
	Token::getAsBool(&cmd, &q);
	dev->setReset(q);
}

void Bg96Driver::funsetPowerKey(OutStream *strm, const char *cmd, void *arg) {
	Bg96Driver *dev = (Bg96Driver*) arg;
	bool q;
	Token::getAsBool(&cmd, &q);
	dev->setPowerKey(q);
}

void Bg96Driver::funSetDtr(OutStream *strm, const char *cmd, void *arg) {
	Bg96Driver *dev = (Bg96Driver*) arg;
	bool q;
	Token::getAsBool(&cmd, &q);
	dev->setDTR(q);
}

void Bg96Driver::funStop(OutStream *strm, const char *cmd, void *arg) {
	Bg96Driver *dev = (Bg96Driver*) arg;

	if (dev->asynch.mRunning) {
		dev->asynch.doStop = true;
		osSignalSet(dev->getThreadId(), Bg96Driver::SIGNAL_BREAK);
		strm->oMsgX(colWHITE, "BG Stop");
	} else {
		strm->oMsgX(colRED, "No in running mode");
	}
}

void Bg96Driver::funStart(OutStream *strm, const char *cmd, void *arg) {
	Bg96Driver *dev = (Bg96Driver*) arg;
	if (!dev->asynch.mRunning) {
		osSignalSet(dev->getThreadId(), Bg96Driver::SIGNAL_BREAK);
		dev->asynch.doStart = true;
		strm->oMsgX(colWHITE, "BG Start");
	} else
		strm->oMsgX(colRED, "No in idle mode");
}

void Bg96Driver::funRestart(OutStream *strm, const char *cmd, void *arg) {
	Bg96Driver *dev = (Bg96Driver*) arg;
	if (dev->asynch.mRunning) {
		dev->asynch.doRestart = true;
	} else
		strm->oMsgX(colRED, "No in running mode");
}

void Bg96Driver::funSetPowerUp(OutStream *strm, const char *cmd, void *arg) {
	Bg96Driver *dev = (Bg96Driver*) arg;
	if (!dev->asynch.mRunning) {
		dev->asynch.doPowerUp = true;
	} else
		strm->oMsgX(colRED, "No in idle mode");
}

void Bg96Driver::funSendCmd(OutStream *strm, const char *cmd, void *arg) {
	Bg96Driver *dev = (Bg96Driver*) arg;
	Token::trim(&cmd);
	dev->writeCmd(cmd);
}

void Bg96Driver::funSetEcho(OutStream *strm, const char *cmd, void *arg) {
	Bg96Driver *dev = (Bg96Driver*) arg;
	Token::trim(&cmd);

	if (dev->setEchoMode(cmd)) {
		strm->oMsgX(colWHITE, "rx:%u tx=%u timeMode=%u logLevel=%u", dev->mEcho.rx, dev->mEcho.tx, dev->mEcho.timeMode, dev->mEcho.logV);
	} else {
		strm->oMsgX(colRED, "format: xxxx");
	}
}

void Bg96Driver::funShowSslCfg(OutStream *strm, const char *cmd, void *arg) {
	Bg96Driver *dev = (Bg96Driver*) arg;
	Token::getAsInt(&cmd, &dev->asynch.sslContext);
	dev->asynch.doShowSSlContext = true;
}

void Bg96Driver::funSetSslCfg(OutStream *strm, const char *cmd, void *arg) {
	Bg96Driver *dev = (Bg96Driver*) arg;
	Token::getAsInt(&cmd, &dev->asynch.sslContext);
	dev->asynch.doSetSSlContext = true;
}

void Bg96Driver::funGetNtp(OutStream *strm, const char *cmd, void *arg) {
	Bg96Driver *dev = (Bg96Driver*) arg;
	dev->asynch.doGetNtpTime = true;
}

void Bg96Driver::funGetGps(OutStream *strm, const char *cmd, void *arg) {
	Bg96Driver *dev = (Bg96Driver*) arg;
	dev->asynch.doGetGps = true;
}

void Bg96Driver::funSendMqtt(OutStream *strm, const char *cmd, void *arg) {
	Bg96Driver *dev = (Bg96Driver*) arg;

	if (Token::getAsInt(&cmd, &dev->asynch.sendMqqtVal)) {
		//jest parametr -> wysłanie MqttSendVar2Name
		dev->asynch.sendMqqtIdx = 1;
	} else {
		//brak parametru -> wysłanie MqttSendVarName
		dev->asynch.sendMqqtIdx = 0;
		dev->asynch.sendMqqtVal = 0;
	}
	dev->asynch.doSendMqtt = true;
}

void Bg96Driver::funOpenMqtt(OutStream *strm, const char *cmd, void *arg) {
	Bg96Driver *dev = (Bg96Driver*) arg;
	dev->asynch.doOpenMqtt = true;
}

void Bg96Driver::funCloseMqtt(OutStream *strm, const char *cmd, void *arg) {
	Bg96Driver *dev = (Bg96Driver*) arg;
	dev->asynch.doCloseMqtt = true;

}

void Bg96Driver::funSendSms(OutStream *strm, const char *cmd, void *arg) {
	Bg96Driver *dev = (Bg96Driver*) arg;
	SendSmsRec *sms = &dev->state.sendSms;

	if (Token::get(&cmd, sms->nrTel, sizeof(sms->nrTel))) {
		Token::get(&cmd, sms->msg, sizeof(sms->msg) - 2);
		Token::remooveQuota(sms->msg);
		int n = strlen(sms->msg);
		sms->msg[n] = 26;
		sms->msg[n + 1] = 0;
		sms->flagSend = true;
		strm->oMsgX(colWHITE, "send sms [%s]", dev->state.sendSms.nrTel);
	}

}

const ShellItemFx menuBG96Fx[] = { //
		{ "s", "stan", Bg96Driver::funShowState }, //
				{ "h", "hardware state", Bg96Driver::funShowHdwState }, //
				{ "i", "informacje", Bg96Driver::funInfo }, //
				{ "gps", "informacje gps", Bg96Driver::funGpsInfo }, //
				{ "3V8", "załącz zasilanie 3.8V", Bg96Driver::funOnOff3_8 }, //
				{ "RstLine", "poziom linii Reset", Bg96Driver::funSetResetLine }, //
				{ "PowerKey", "poziom linii PowerKey", Bg96Driver::funsetPowerKey }, //
				{ "DTR", "poziom linii DTR", Bg96Driver::funSetDtr }, //
				{ "stop", "zatrzymanie obsługi", Bg96Driver::funStop }, //
				{ "start", "start obsługi", Bg96Driver::funStart }, //
				{ "restart", "restart obsługi modemu", Bg96Driver::funRestart }, //
				{ "PowerUp", "sekwencja włączania", Bg96Driver::funSetPowerUp }, //
				{ "w", "wysłanie komendy do modemu", Bg96Driver::funSendCmd }, //
				{ "echo", "echo: rx,tx,time_mode:off,from_send,from_start. Np. 112", Bg96Driver::funSetEcho }, //
				{ "sslcfg", "pokaż parametry kontekstu ssl", Bg96Driver::funShowSslCfg }, //
				{ "setsslcfg", "ustaw konfigurację SSL", Bg96Driver::funSetSslCfg }, //
				{ "get_ntp", "pobierz czas NTP", Bg96Driver::funGetNtp }, //
				{ "get_gps", "pobierz dane GPS", Bg96Driver::funGetGps }, //
				{ "send_mqtt", "send mqtt", Bg96Driver::funSendMqtt }, //
				{ "open_mqtt", "open mqtt", Bg96Driver::funOpenMqtt }, //
				{ "close_mqtt", "close mqtt", Bg96Driver::funCloseMqtt }, //
				{ "sms", "wyślij sms: numer_dst, treść", Bg96Driver::funSendSms }, //
				{ NULL, NULL } };

void Bg96Driver::shell(OutStream *strm, const char *cmd) {

	execMenuCmd(strm, menuBG96Fx, cmd, this, "BG96 Menu");
}

