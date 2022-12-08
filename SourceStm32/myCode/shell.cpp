/*
 * shell.cpp
 *
 *  Created on: Dec 5, 2020
 *      Author: Grzegorz
 */


#include "lwip.h"
#include "dns.h"
#include "icmp.h"
#include "sockets.h"
#include "inet_chksum.h"

#include "shell.h"
#include "EscTerminal.h"
#include "utils.h"
#include "ethernetif.h"
#include "Bg96Driver.h"
#include "DustSensorBase.h"
#include "GlobData.h"
#include "I2cDev.h"
#include "UMain.h"
#include "MdbMasterTask.h"
#include "ProjectConfig.h"
#include "Hal.h"

#include "_SensorDrivers.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>


extern Bg96Driver *bg96;
extern DustSensorBase *dustInternSensor;
extern MdbMasterDustTask *dustExternSensor;
extern NoiseDetector *mdbMaster_1;
extern MdbMasterTask *mdbMaster_2;
extern SHT35DevPub *sht35;
extern Bmp338DevPub *bmp338;
extern LedMatrix *ledMatrix;

//-------------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------------
// ShellConnection
//-------------------------------------------------------------------------------------------------------------------------
ShellConnection::ShellConnection() {
	mThreadId = NULL;
	txBuf = new RxTxBuf(TX_BUF_SIZE);
	rxRec.head = 0;
	rxRec.tail = 0;
	mReStartCnt = 0;
}
void ShellConnection::startSendNextPart() {
	const char *ptr;
	int cnt;

	bool q = txBuf->getLinearPart(&ptr, &cnt);
	if (q) {
		sendBuf(ptr, cnt);
	}
}

//funkcja zwraca TRUE jeśli udało się wszystko umieścić w buforze
bool ShellConnection::writeData(Portion *portion) {
	if (portion->len <= 0)
		return true;
	if (!isReady())
		return true;
	bool q = txBuf->addBuf(portion);
	if (!isConnSending()) {
		startSendNextPart();
	}
	return q;
}

bool ShellConnection::getChar(char *ch) {
	if (rxRec.head != rxRec.tail) {
		int t = rxRec.tail;
		*ch = rxRec.buf[t];
		if (++t >= (int) sizeof(rxRec.buf))
			t = 0;
		rxRec.tail = t;
		return true;
	}
	return false;
}
//-------------------------------------------------------------------------------------------------------------------------
// UartConnection
//-------------------------------------------------------------------------------------------------------------------------
UartConnection::UartConnection() :
		TUart::TUart(TUart::myUART4, 7), ShellConnection::ShellConnection() {
	memset(&rxRec, 0, sizeof(rxRec));
}

bool UartConnection::isConnSending() {
	return isSending();
}

void UartConnection::sendBuf(const void *ptr, int len) {
	TUart::writeBuf(ptr, len);
}

void UartConnection::TxCpltCallback() {
	TUart::TxCpltCallback();
	if (isRxOverrun()) {
		uartAbort();
		StartRecive();
		mReStartCnt++;
	}
	startSendNextPart();
}

void UartConnection::StartRecive() {
	HAL_UART_Receive_IT(&mHuart, (uint8_t*) &rxChar, 1);
}

void UartConnection::RxCpltCallback() {
	rxRec.buf[rxRec.head] = rxChar;
	if (++rxRec.head >= (int) sizeof(rxRec.buf))
		rxRec.head = 0;
	if (mThreadId != NULL) {
		osSignalSet(mThreadId, ShellTask::SIGNAL_CHAR);
	}
	StartRecive();
}

void UartConnection::ErrorCallback() {

}

HAL_StatusTypeDef UartConnection::Init() {
	HAL_StatusTypeDef st = TUart::Init(115200);
	if (st == HAL_OK) {
		StartRecive();
	}
	return st;
}

//-------------------------------------------------------------------------------------------------------------------------
// USBConnection
//-------------------------------------------------------------------------------------------------------------------------
USBConnection *USBConnection::Me = NULL;
extern "C" uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len);
extern "C" uint8_t CDC_IsSending(void);

extern "C" void CDC_UserRecivedData(uint8_t *Buf, uint32_t Len) {
	if (USBConnection::Me != NULL) {
		USBConnection::Me->inpDataFun(Buf, Len);
	}
}

extern "C" void CDC_UserTransmitCplt(void) {
	if (USBConnection::Me != NULL) {
		USBConnection::Me->transmitCplt();
	}
}

extern "C" void CDC_UserInit(void) {
	if (USBConnection::Me != NULL) {
		USBConnection::Me->initCDC();
	}

}

USBConnection::USBConnection() {
	Me = this;
	mReady = false;
	mTransmitTick = 0;
	rxRec.head = 0;
	rxRec.tail = 0;
}

HAL_StatusTypeDef USBConnection::Init() {
	return HAL_OK;
}

bool USBConnection::isConnSending() {
	return CDC_IsSending();
}

bool USBConnection::isReady() {
	if (!mReady)
		return false;
	if (mTransmitTick != 0) {
		if (HAL_GetTick() - mTransmitTick > 1000) {
			mTransmitTick = 0;
			mReady = false;
			return false;
		}
	}
	return true;
}

void USBConnection::sendBuf(const void *ptr, int len) {
	if (mReady) {
		CDC_Transmit_FS((uint8_t*) ptr, len);
		mTransmitTick = HAL_GetTick();
	}
}

void USBConnection::initCDC() {
	mReady = true;
	mTransmitTick = 0;
	txBuf->clear();
}

void USBConnection::transmitCplt() {
	mTransmitTick = 0;
	startSendNextPart();
}

void USBConnection::inpDataFun(uint8_t *Buf, uint32_t Len) {
	if (Len > 0) {
		for (uint32_t i = 0; i < Len; i++) {
			rxRec.buf[rxRec.head] = Buf[i];
			if (++rxRec.head >= (int) sizeof(rxRec.buf))
				rxRec.head = 0;
		}
		if (mThreadId != NULL) {
			osSignalSet(mThreadId, ShellTask::SIGNAL_CHAR);
		}
	}

}

//-------------------------------------------------------------------------------------------------------------------------
// ShellTask
//-------------------------------------------------------------------------------------------------------------------------
ShellTask *ShellTask::Me = NULL;

ShellTask::ShellTask() :
		TaskClass::TaskClass("Shell", osPriorityNormal, 1024) {

	Me = this;

	osMutexDef(OutStr);
	mOutTxtMutex = osMutexCreate(osMutex(OutStr));
	mNoTermSmfCnt = 0;
	mFullTxCnt = 0;

	ST3 pin0 = Hdw::getPinCfg0();

	// jeśli załaczona obsługa LedMatrix to nie może być obsługi shell na UART4
	if (config->data.R.rest.ledMatrixRun) {
		if (pin0 == posGND) {
			pin0 = posFREE;
		}
	}

	switch (pin0) {
	default:
	case posFREE:
	case posVCC:
		myConnection = new USBConnection();
		break;
	case posGND:
		myConnection = new UartConnection();
	}
	term = new EscTerminal(this);
	myConnection->Init();
}

void ShellTask::_Msg(TermColor color, const char *pFormat, ...) {
	if (Me != NULL) {
		va_list ap;
		va_start(ap, pFormat);
		Me->oMsgX(color, pFormat, ap);
		va_end(ap);
	}
}

void ShellTask::putOut(const void *mem, int len) {
	bool full = 0;
	Portion portion;
	portion.len = len;
	portion.dt = (const char*) mem;

	while (1) {
		bool q = myConnection->writeData(&portion);
		if (q) {
			break;
		} else {
			if (!full) {
				full = 0;
				mFullTxCnt++;
			}
		}
		osDelay(5); // blokowanie tasku wysyłającego
	}
}

void ShellTask::putOutStr(const char *str) {
	flgSendAny = true;
	putOut(str, strlen(str));

}

bool ShellTask::openOutMutex(int tm) {
	return (osMutexWait(mOutTxtMutex, tm) == osOK);
}
void ShellTask::closeOutMutex() {
	osMutexRelease(mOutTxtMutex);
}

void ShellTask::ThreadFunc() {

	myConnection->setThreadId(getThreadId());
	putOutStr(TERM_CLEAR_SCR);
	oMsgX(colCYAN, "Welcome. start=%d\r\nUżywaj PuTTY jako terminala !!!", nir.startCnt);

	xEventGroupSetBits(sysEvents, EVENT_TERM_RDY);

	// odczeakanie az DefaultTask utworzy resztę urządzeń logicznych

	xEventGroupWaitBits(sysEvents, EVENT_CREATE_DEVICES, false, false, 1000000);
	oMsgX(colCYAN, "Ready");

	while (1) {
		osEvent ev = osSignalWait(SIGNAL_CHAR | SIGNAL_MSG, 1000);
		imAlive();
		if (ev.status == osEventSignal) {
			int code = ev.value.v;

			if (code == SIGNAL_CHAR) {
				char key;
				while (myConnection->getChar(&key)) {
					TermAct act = term->inpChar(key);
					switch (act) {
					case actNOTHING:
						break;
					case actLINE:
						flgSendAny = false;
						execCmdLine(term->mCmd);
						if (!flgSendAny)
							term->showLineMx();
						break;
					case actALTCHAR:
						execAltChar(term->mAltChar);
						break;
					case actFUNKEY:
						execFunKey(term->mFunKey);
						break;
					}
				}
			} else if (code == SIGNAL_MSG) {

			}
		}
	}
}

const char* ShellTask::getColorStr(TermColor color) {
	switch (color) {
	default:
	case colWHITE:
		return TERM_COLOR_WHITE;
	case colRED:
		return TERM_COLOR_RED;
	case colGREEN:
		return TERM_COLOR_GREEN;
	case colBLUE:
		return TERM_COLOR_BLUE;
	case colMAGENTA:
		return TERM_COLOR_MAGENTA;
	case colYELLOW:
		return TERM_COLOR_YELLOW;
	case colCYAN:
		return TERM_COLOR_CYAN;
	}
}

void ShellTask::oFormatX(TermColor color, const char *pFormat, va_list ap) {
	if (oOpen(color)) {
		int len = vsnprintf(outBuf, sizeof(outBuf), pFormat, ap);
		putOut(outBuf, len);
		putOutStr("\r\n");
		oClose();
	} else {
		mNoTermSmfCnt++;
	}

}

void ShellTask::oMsgX(TermColor color, const char *pFormat, ...) {
	va_list ap;
	va_start(ap, pFormat);
	oFormatX(color, pFormat, ap);
	va_end(ap);
}

bool ShellTask::oOpen(TermColor color) {
	bool q = openOutMutex(TermStream::STD_TIME);
	if (q) {
		putOutStr(TERM_CLEAR_LINE);
		putOutStr(getColorStr(color));
	}
	return q;
}
void ShellTask::oClose() {
	term->showLineNoMx();
	closeOutMutex();
}

void ShellTask::oWr(const char *txt) {
	int len = strlen(txt);
	putOut(txt, len);
	putOutStr("\r\n");
}

void ShellTask::oMsg(const char *pFormat, ...) {
	va_list ap;
	va_start(ap, pFormat);
	int len = vsnprintf(outBuf, sizeof(outBuf), pFormat, ap);
	va_end(ap);
	putOut(outBuf, len);
	putOutStr("\r\n");
}

void ShellTask::dumpBuf(TermColor color, const char *buf) {
	if (oOpen(color)) {
		int len = strlen(buf);
		putOut(buf, len);
		oClose();
	} else {
		mNoTermSmfCnt++;
	}
}

void ShellTask::execAltChar(char altChar) {
	char buf[40];
	snprintf(buf, sizeof(buf), "AltChar=%u [%c]", altChar, altChar);
	oMsgX(colRED, buf);
}

extern "C" void StartMeasureAdc1();
extern "C" void getTempNtcEx(float *tab);




static void funShowState(){

}



const ShellItem mainMenu[] = { //
		{ "s", "[F2] status urządzenia" }, //
				{ "h", "[F3] stan hardware" }, //
				{ "reboot", "reboot STM" }, //
				{ "cfg", ">> menu konfiguracji" }, //
				{ "time", ">> menu czasu" }, //
				{ "eth", ">> menu etherneta" }, //
				{ "net", ">> menu tcp/ip" }, //
				{ "bg", ">> menu BG96" }, //
				{ "iic", ">> menu układów i2c" }, //
				{ "dust", ">> menu czujnika pyłów" }, //
				{ "noise", ">> czujnik hałasu, menu modbus master X7" }, //
				{ "gas", ">> czujnik  gazów, menu modbus master X6" }, //
				{ "matrix", ">> menu LedMatrix" }, //
				{ "ps", "lista wątków" }, //
				{ "psx", "lista tasków" }, //
				{ "mem", "informacja o pamięci" }, //
				{ "glob", "dane globalne" }, //
				{ "jglob", "dane globalne w postaci json" }, //
				{ "globdef", "definicja danych globalnych" }, //
				{ "lcd_scr", "ustawienie numer wyświetlanego ekranu" }, //
				{ "lcd_time", "ustawienie czasu przełaczania ekranów na lcd" }, //
				{ "ntc", "pokaż temperaturę z NTC" }, //

				{ NULL, NULL } };

void ShellTask::execCmdLine(const char *cmd) {

	char tok[20];
	int idx = -1;
	if (Token::get(&cmd, tok, sizeof(tok)))
		idx = findCmd(mainMenu, tok);
	switch (idx) {
	case 0:
		showDevState();
		break;
	case 1:
		showHdwState();
		break;
	case 2:
		oMsgX(colRED, "*** R E S E T ***");
		reboot(1000);
		break;
	case 3:
		config->shell(this, cmd);
		break;
	case 4:
		timeMenu(cmd);
		break;
	case 5:
		ethMenu(cmd);
		break;
	case 6:
		netMenu(cmd);
		break;
	case 7:
		bg96->shell(this, cmd);
		flgSendAny = true;
		break;
	case 8: //iic
		I2c1Bus::shell(this, cmd);
		break;
	case 9: //dust
		if (config->data.P.dustInpType == dust_Intern) {
			dustInternSensor->shell(this, cmd);
		} else {
			dustExternSensor->shell(this, cmd);
		}
		break;

	case 10: //noise - mdb1
		mdbMaster_1->shell(this, cmd);
		break;
	case 11: //gas - mdb2
		mdbMaster_2->shell(this, cmd);
		break;

	case 12: //matrix
		if (ledMatrix != NULL) {
			ledMatrix->shell(this, cmd);
		} else {
			oMsgX(colRED, "Obsluga LedMatrix wylaczona");
		}
		break;

//--------------------------
	case 13: //ps
		showThreadList();
		break;
	case 14: //ps
		TaskClassList::ShowList(this);
		break;

	case 15: //mem
		showMemInfo();
		break;

	case 16: //glob
		GlobData::show(this);
		break;
	case 17: //jglob
		GlobData::showJson(this);
		break;
	case 18: //globdef
		GlobData::showDef(this);
		break;

	case 19: { //lcd_scr
		int v;
		Token::getAsInt(&cmd, &v);
		setLcdScrNr(v);
	}
		break;
	case 20: { //lcd_time
		int v;
		Token::getAsInt(&cmd, &v);
		setLcdTime(v);
	}
		break;
	case 21: { //ntc
		NTC::StartMeasure();
		if (NTC::WaitForMeasEnd(500)) {
			oMsgX(colWHITE, "NTC: U=%.3f[V] R=%.2f[kom] temp=%.1f[deg]", NTC::nap, NTC::rez / 1000, NTC::temp);
		}
	}
		break;


	default: {
		char txt[100];
		snprintf(txt, sizeof(txt), "AIR-PRO: MainMenu, NS=%s", config->data.P.SerialNr);
		showHelp(this, txt, mainMenu);
	}
		break;
	};

}

void *mem_try = NULL;

extern uint8_t _end; /* Symbol defined in the linker script */
extern uint8_t _estack; /* Symbol defined in the linker script */
extern uint32_t _Min_Stack_Size; /* Symbol defined in the linker script */

void ShellTask::showMemInfo() {
	if (oOpen(colWHITE)) {
		if (mem_try == NULL)
			mem_try = malloc(4);

		const uint32_t stack_start = (uint32_t) &_end;
		const uint32_t stack_limit = (uint32_t) &_estack - (uint32_t) &_Min_Stack_Size;

		oMsg("heap_begin= %p", stack_start);
		oMsg("heap_end= %p", stack_limit);
		oMsg("heap_curr = %p", mem_try);

		int size = stack_limit - stack_start;
		int used = (int) mem_try - stack_start;
		int freem = size - used;
		int stack_sz = (uint32_t) &_Min_Stack_Size;

		oMsg("heap_size = %u (%p)", size, size);
		oMsg("heap_used = %u (%p)", used, used);
		oMsg("heap_free = %u (%p)", freem, freem);
		oMsg("stack_size = %u (%p)", stack_sz, stack_sz);

		oClose();
	}
}
void ShellTask::showThreadList() {
	if (oOpen(colWHITE)) {
		char *bigbuf;
		bigbuf = (char*) malloc(2000);
		if (bigbuf != NULL) {
			vTaskList(bigbuf);
			oMsg("TXT_SIZE=%u (max %u) (p=%08X)", strlen(bigbuf), 2000, (int) bigbuf);
			oMsg("Name\t\tState\tPrior.\tStackP\tNum");
			oWr(bigbuf);
			free(bigbuf);
		} else
			oMsg("NoFreeMem for Buffer");

		oClose();
	}
}

void ShellTask::showHdwState() {
	static int showCnt;
	if (oOpen(colYELLOW)) {
		oMsg("--- %u ----------", showCnt++);
		oMsg("CFG0:%s", ST3Str(Hdw::getPinCfg0()));
		oMsg("CFG1:%s", ST3Str(Hdw::getPinCfg1()));
		oMsg("CFG2:%s", ST3Str(Hdw::getPinCfg2()));
		oMsg("CFG3:%s", ST3Str(Hdw::getPinCfg3()));
		oMsg("DUST_ON:%s", YN(Hdw::getDustSensorOn()));
		oMsg("DUST_FLG:%s", ErrOk(Hdw::getDustSensorFlg()));
		oMsg("HEATER_ON:%s", YN(Hdw::getHeaterOn()));
		oMsg("HEATER_FLG:%s", ErrOk(Hdw::getHeaterFlg()));
		oClose();
	}

}
void ShellTask::showDevState() {
	static int showCnt;
	if (oOpen(colWHITE)) {
		oMsg("--- %u ----------", showCnt++);
		char buf[20];
		TimeTools::DtTmStr(buf, &mSoftVer.time);
		oMsg("Ver             :%u.%03u - %s", mSoftVer.ver, mSoftVer.rev, buf);
		oMsg("RtcInitStatus   :%s", HAL_getErrStr(Rtc::mRtcStatus));
		if (config->data.P.dustInpType == dust_Intern) {
			oMsg("DustInternSensor:%s", ErrOk(dustInternSensor->isError()));
		} else {
			oMsg("DustExternSensor:%s", ErrOk(dustExternSensor->isError()));
		}
		if (mdbMaster_1->isCfgNoiseOn()) {
			oMsg("NoiseSensor     :%s", ErrOk(mdbMaster_1->isError()));
		}
		if (mdbMaster_2->isAnyConfiguredData()) {
			oMsg("GasSensor       :%s", ErrOk(mdbMaster_2->isDataError()));
		}
		oMsg("Bmp338          :%s", ErrOk(bmp338->isError()));
		oMsg("Sht35           :%s", ErrOk(sht35->isError()));
		oMsg("SIM card rdy    :%s", YN(bg96->isSimCardInserted()));
		oMsg("Network regist. :%s", YN(bg96->isNetworkRegistered()));
		oMsg("Network IP rdy  :%s", YN(bg96->isIPready()));
		oMsg("MQTT svr opened :%s", YN(bg96->isMqttSvrOpened()));
		oMsg("MQTT Send       :%s", YN(bg96->isMqttSendingOk()));
		oMsg("-----");
		oMsg("Term no semafor :%u", mNoTermSmfCnt);
		oMsg("Term full TX buf:%u", mFullTxCnt);
		oMsg("RestartRxCnt    :%u", myConnection->mReStartCnt);

		oClose();
	}
}

void ShellTask::execFunKey(FunKey funKey) {
	//msg(colYELLOW, "FunKey=%u", funKey);
	switch (funKey) {
	default:
	case fnF1:
		showHelp(this, "MainMenu", mainMenu);
		break;
	case fnF2:
		showDevState();
		break;
	case fnF3:
		showHdwState();
		break;

	}
}

//--------EthMenu-----------------------------------------------------------------
const ShellItem menuEth[] = { //
		{ "s", "status etherneta" }, //
				{ "reset", "impuls reset do PHY" }, //
				{ "pwr", "=0 (wyłącz), =1(załącz) zasilanie PHY" }, //
				{ "reg", "pokaż rejestry PHY" }, //
				{ NULL, NULL } };

typedef struct {
	const char *name;
	int adr;
} PhyRegItemDef;
const PhyRegItemDef phyRegTab[] = { //
		{ "BCR", 0 }, //
				{ "BSR", 1 }, //
				{ "ID1", 2 }, //
				{ "ID2", 3 }, //
				{ "NegAdver", 4 }, //
				{ "NegPartner", 5 }, //
				{ "NegExpan", 6 }, //
				{ "Mode Control/Status Reg", 17 }, //
				{ "Special Mode Reg", 18 }, //
				{ "SymbolErrCntReg", 26 }, //
				{ "Special Control/Status Indications Register", 27 }, //
				{ "Interrupt Source Flag Register", 29 }, //
				{ "Interrupt Mask Register", 30 }, //
				{ "PHY Special Control/Status Register", 31 }, //
				{ NULL, -1 }, };

extern "C" void ethGetPhyReg(const uint16_t *adrTab, uint16_t *valTab);

void ShellTask::ethMenu(const char *cmd) {
	char tok[20];
	int idx = -1;
	if (Token::get(&cmd, tok, sizeof(tok)))
		idx = findCmd(menuEth, tok);
	switch (idx) {
	case 0:
		break;
	case 1:
		oMsgX(colWHITE, "PHY Reset");
		Hdw::phyReset(1);
		osDelay(50);
		Hdw::phyReset(0);
		break;

	case 2: {
		int val;
		if (Token::getAsInt(&cmd, &val)) {
			oMsgX(colWHITE, "PHY power = %d", val);
			Hdw::phyPower(val != 0);
		}
	}
		break;

	case 3: {
		uint16_t adrTab[30];
		uint16_t valTab[30];
		int k = 0;
		while (phyRegTab[k].name != NULL) {
			adrTab[k] = phyRegTab[k].adr;
			k++;
		}
		adrTab[k] = 0xFFFF;
		ethGetPhyReg(adrTab, valTab);
		if (oOpen(colWHITE)) {
			int k = 0;
			while (phyRegTab[k].name != NULL) {
				oMsg("%2u. %04X %s", phyRegTab[k].adr, valTab[k], phyRegTab[k].name);
				k++;
			}
			oClose();
		}

	}
		break;

	default:
		showHelp(this, "Ethernet Menu", menuEth);
		break;
	};
}

//--------TimeMenu-----------------------------------------------------------------
const ShellItem menuTime[] = { //
		{ "s", "pokaż czas" }, //
				{ "i", "pokaż informacje" }, //
				{ "settime", "ustaw czas - gg:mm:ss" }, //
				{ "setdate", "ustaw date - rrrr.mm.dd" }, //
				{ "init", "init RTC" }, //
				{ NULL, NULL } };

void ShellTask::timeMenu(const char *cmd) {
	char tok[20];
	int idx = -1;
	if (Token::get(&cmd, tok, sizeof(tok)))
		idx = findCmd(menuTime, tok);
	switch (idx) {
	case 0: //s
	{
		TDATE tm;
		char buf[30];
		if (Rtc::ReadTime(&tm)) {
			TimeTools::DtTmStrZZ(buf, &tm);
			oMsgX(colGREEN, buf);
		} else {
			oMsgX(colRED, "Błąd pobrania czasu");
		}
	}
		break;
	case 1: //i
	{
		TDATE tm;
		char buf[30];
		if (Rtc::ReadTime(&tm)) {
			TimeTools::DtTmStrZZ(buf, &tm);
			oMsgX(colGREEN, "Czas:%s", buf);
		} else {
			oMsgX(colRED, "Błąd pobrania czasu");
		}
		TimeTools::DtTmStrZZ(buf, &config->data.R.rtcSetUpTime);
		oMsgX(colGREEN, "Czas ustawienia czasu: %s", buf);
		oMsgX(colGREEN, "Zródło ustawienia czasu: %s", getTmSrcName(config->data.R.rtcSetUpTime.timeSource));

	}
		break;
	case 2: //settime
	{
		TDATE tm;
		if (TimeTools::parseTime(&cmd, &tm)) {
			if (Rtc::SetTime(&tm)) {
				Rtc::ReadTime(&tm);
				tm.timeSource = tmSrcNTP;
				config->data.R.rtcSetUpTime = tm;
				config->saveRtc();
				oMsgX(colGREEN, "Ok");
			} else
				oMsgX(colRED, "Set time error");
		} else
			oMsgX(colRED, "Time format error");
	}
		break;
	case 3: //setdate
	{
		TDATE tm;
		if (TimeTools::parseDate(&cmd, &tm)) {
			if (Rtc::SetDate(&tm)) {
				Rtc::ReadTime(&tm);
				tm.timeSource = tmSrcNTP;
				config->data.R.rtcSetUpTime = tm;
				config->saveRtc();
				oMsgX(colGREEN, "Ok");
			} else
				oMsgX(colRED, "Set date error");
		} else
			oMsgX(colRED, "Date format error");
	}
		break;

	case 4: //init
	{
		Rtc::Init();
		oMsgX(colWHITE, "RtcInitStatus   :%s", HAL_getErrStr(Rtc::mRtcStatus));
	}
		break;
	default:
		showHelp(this, "Time Menu", menuTime);
		break;
	};
}

//--------NetMenu-----------------------------------------------------------------
const ShellItem menuNet[] = { //
		{ "s", "pokaż stan" }, //
				{ "restart", "rekonfiguruj net" }, //
				{ "getip", "użyj DNS" }, //
				//{ "ping", "ping" }, //
				{ NULL, NULL } };

ip_addr_t globAddr;

void dns_found_cb(const char *name, const ip_addr_t *ipaddr, void *callback_arg) {
	if (ipaddr != NULL && ipaddr->addr != 0) {
		char txt[20];
		ipaddr_ntoa_r(ipaddr, txt, sizeof(txt));
		ShellTask::_Msg(colGREEN, "%s -> %s", name, txt);
	} else {
		ShellTask::_Msg(colRED, "%s -> ???", name);
	}
}

void ShellTask::netMenu(const char *cmd) {
	char tok[20];
	int idx = -1;

	if (Token::get(&cmd, tok, sizeof(tok)))
		idx = findCmd(menuNet, tok);
	switch (idx) {
	case 0: //s
	{
		NetState netState;
		char txt[20];

		getNetIfState(&netState);
		if (oOpen(colWHITE)) {
			oMsg("LinkUp:%s", YN(netState.LinkUp));
			oMsg("Dhcp:%s", OnOff(netState.DhcpOn));
			oMsg("DhcpRdy:%s", YN(netState.DhcpRdy));
			if (netState.ipValid) {
				ipaddr_ntoa_r(&netState.CurrIP, txt, sizeof(txt));
				oMsg("IP:%s", txt);
				ipaddr_ntoa_r(&netState.CurrMask, txt, sizeof(txt));
				oMsg("Mask:%s", txt);
				ipaddr_ntoa_r(&netState.CurrGate, txt, sizeof(txt));
				oMsg("GateWay:%s", txt);
			}
			const ip_addr_t *pdns1 = dns_getserver(0);
			ipaddr_ntoa_r(pdns1, txt, sizeof(txt));
			oMsg("DNS_1:%s", txt);
			const ip_addr_t *pdns2 = dns_getserver(1);
			ipaddr_ntoa_r(pdns2, txt, sizeof(txt));
			oMsg("DNS_2:%s", txt);

			oClose();
		}

	}

		break;
	case 1: //restart
		oMsgX(colWHITE, "Network reconfig");
		reconfigNet();

		break;
	case 2: //getip
	{
		Token::trim(&cmd);
		int err = dns_gethostbyname(cmd, &globAddr, &dns_found_cb, NULL);
		switch (err) {
		case ERR_OK: //
		{
			char txt[20];
			ipaddr_ntoa_r(&globAddr, txt, sizeof(txt));
			oMsgX(colGREEN, "IME %s -> %s", cmd, txt);
		}
			break;
		case ERR_INPROGRESS:
			break;
		default:
			oMsgX(colRED, "Error :%d", err);
		}

	}
		break;
	case 3: //ping
	{
		Token::trim(&cmd);

		ip4_addr_t addr;
		if (ipaddr_aton(cmd, &addr)) {
			if (oOpen(colWHITE)) {
				Ping(addr, 32);
				oClose();
			}
		}
	}
		break;
	default:
		showHelp(this, "Net Menu", menuNet);
		break;
	};
}

//-----------------------------------------------------------------------------------

/**
 * return number of milliseconds since boot time (or any other reference)
 */
u32_t sys_start_tickcount() {
	return HAL_GetTick();
}

/**
 * return number of milliseconds since "ref"
 */
u32_t sys_stop_tickcount(u32_t ref) {
	return HAL_GetTick() - ref;

}
#define PING_TIMEOUT 1000
/*----------------------------------------------------------------------------*/
int ShellTask::Ping(ip4_addr_t addr, int length)
/*----------------------------------------------------------------------------*/
{ /* Variables de travail */
	int iResult = 0;
	static int seq_num = 0;
	int timeout = PING_TIMEOUT;
	int err = 0;
	u32_t ulTickCount;
	char reply[64];
	struct icmp_echo_hdr *pecho;
	struct icmp_echo_hdr *pechoreply = (struct icmp_echo_hdr*) reply;
	struct sockaddr_in saLocal;
	int fromlen;
	struct sockaddr_in from;

	char txt[20];
	ipaddr_ntoa_r(&addr, txt, sizeof(txt));

	oMsg("Ping %s : bytes=%i\n", txt, length);

	if (!(pecho = (struct icmp_echo_hdr*) malloc(sizeof(struct icmp_echo_hdr) + length)))
		return ENOMEM;

	for (int i = 0; i < length; i++) {
		((char*) (pecho))[sizeof(struct icmp_echo_hdr) + i] = ('a' + (i % 26));
	}

	/* Ouverture d'une socket en mode "raw" pour ICMP */
	int hSocket = socket(AF_INET, SOCK_RAW, IP_PROTO_ICMP);
	if (hSocket >= 0) {
		memset(&saLocal, 0, sizeof(saLocal));
		saLocal.sin_family = AF_INET;
		saLocal.sin_port = htons(INADDR_ANY);
		saLocal.sin_addr.s_addr = htons(INADDR_ANY);

		err = setsockopt(hSocket, SOL_SOCKET, SO_RCVTIMEO, (char* ) &timeout, sizeof(timeout));

		/* Liaison à l'Adresse Locale */
		if (bind(hSocket, (struct sockaddr*) &saLocal, sizeof(saLocal)) == 0) { /* Paramétrage du timeout de réception */

			/* Préparation de la trame ICMP "Echo" */
			ICMPH_TYPE_SET(pecho, ICMP_ECHO);
			pecho->chksum = 0;
			pecho->id = htons(0x1234); /* Pour repérer ces "Pings" */
			pecho->seqno = htons(seq_num++);
			pecho->chksum = inet_chksum(pecho, sizeof(struct icmp_echo_hdr) + length);

			struct sockaddr_in dstAddr;

			memset(&dstAddr, 0, sizeof(dstAddr));
			dstAddr.sin_len = sizeof(dstAddr);
			dstAddr.sin_family = AF_INET;
			dstAddr.sin_port = htons(INADDR_ANY);
			dstAddr.sin_addr.s_addr = addr.addr;

			/* Si le "ping" est bien envoyé... */
			if ((err = sendto(hSocket, pecho, sizeof(struct icmp_echo_hdr) + length, 0, (struct sockaddr* ) &dstAddr, sizeof(dstAddr))) >= 0) { /* Réception de l' "Echo" (avec mesure du délai en ms) */
				ulTickCount = sys_start_tickcount();
				do {
					osDelay(5);
					err = recvfrom(hSocket, reply, sizeof(reply), 0, (struct sockaddr* ) &from, (socklen_t* ) &fromlen);

				} while ((err > 0) && ((pechoreply->id != pecho->id) || (pechoreply->seqno != pecho->seqno)) && (sys_stop_tickcount(ulTickCount) <= timeout));
				ulTickCount = sys_stop_tickcount(ulTickCount);

				/* Si le "ping" est bien reçu... */
				if (err > 0) { /* Affichage dans la console */
					ipaddr_ntoa_r(&addr, txt, sizeof(txt));

					oMsg("Ping Reply from %s : bytes=%i, delay=%lums\n", txt, (err - sizeof(struct icmp_echo_hdr))/*length*/, ulTickCount);
					iResult = 0;
				}
				/* Si on a eu une "non-réception"... */
				else { /* Affichage dans la console */
					oMsg("Ping Request timed out...\n");
					iResult = ETIMEDOUT;
				}
			} else {
				iResult = errno;
				oMsg("Ping sendto=%i\n", iResult);
			}
		}

		closesocket(hSocket);
	} else {
		iResult = errno;
		oMsg("Ping socket=%d\n", iResult);
	}

	free(pecho);

	return iResult;
}
