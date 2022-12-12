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
#include "Token.h"

#include "_SensorDrivers.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

extern Bg96Driver *bg96;
extern DustSensorBase *dustInternSensor;
extern ExtDustsensor *dustExternSensor;
extern MdbMasterTask *mdbMaster_1;
extern MdbMasterTask *mdbMaster_2;
extern SHT35Device *sht35;
extern Bmp338Device *bmp338;
extern LedMatrix *ledMatrix;
extern I2cBus *i2cBus1;
extern GasS873 *gasS873;
extern NoiseDetector *noiseDet;


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
						execCmdLineEx(term->mCmd);
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
	bool q = openOutMutex(OutHdStream::STD_TIME);
	if (q) {
		putOutStr(TERM_CLEAR_LINE);
		putOutStr(term->getColorStr(color));
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

extern const ShellItemFx mainMenuFx[];
static void funShowState(OutStream *strm, const char *cmd, void *arg);
static void funShowHardware(OutStream *strm, const char *cmd, void *arg);

void ShellTask::execCmdLineEx(const char *cmd) {
	char txt[100];
	snprintf(txt, sizeof(txt), "AIR-PRO: MainMenu, NS=%s", config->data.P.SerialNr);

	execMenuCmd(this, mainMenuFx, cmd, this, txt);
}

void ShellTask::execFunKey(FunKey funKey) {
	//msg(colYELLOW, "FunKey=%u", funKey);
	switch (funKey) {
	default:
	case fnF1:
		showHelpFx(this, "MainMenu", mainMenuFx);
		break;
	case fnF2:
		funShowState(this, NULL, NULL);
		break;
	case fnF3:
		funShowHardware(this, NULL, NULL);
		break;

	}
}

//-----------------------------------------------------------------------------------------------------------------------
// Main Menu
//-----------------------------------------------------------------------------------------------------------------------

static void funShowState(OutStream *strm, const char *cmd, void *arg) {
	static int showCnt;
	if (strm->oOpen(colWHITE)) {
		strm->oMsg("--- %u ----------", showCnt++);
		char buf[20];
		TimeTools::DtTmStr(buf, &mSoftVer.time);
		strm->oMsg("Ver             :%u.%03u - %s", mSoftVer.ver, mSoftVer.rev, buf);
		strm->oMsg("RtcInitStatus   :%s", HAL_getErrStr(Rtc::mRtcStatus));
		if (config->data.P.dustInpType == dust_Intern) {
			strm->oMsg("DustInternSensor:%s", ErrOk(dustInternSensor->isDataError()));
		} else {
			strm->oMsg("DustExternSensor:%s", ErrOk(dustExternSensor->isDataError()));
		}
		if (noiseDet!=NULL) {
			strm->oMsg("NoiseSensor     :%s", ErrOk(noiseDet->isDataError()));
		}
		if (gasS873->isAnyConfiguredData()) {
			strm->oMsg("GasSensor       :%s", ErrOk(gasS873->isDataError()));
		}
		strm->oMsg("Bmp338          :%s", ErrOk(bmp338->isDataError()));
		strm->oMsg("Sht35           :%s", ErrOk(sht35->isDataError()));
		strm->oMsg("SIM card rdy    :%s", YN(bg96->isSimCardInserted()));
		strm->oMsg("Network regist. :%s", YN(bg96->isNetworkRegistered()));
		strm->oMsg("Network IP rdy  :%s", YN(bg96->isIPready()));
		strm->oMsg("MQTT svr opened :%s", YN(bg96->isMqttSvrOpened()));
		strm->oMsg("MQTT Send       :%s", YN(bg96->isMqttSendingOk()));
		strm->oMsg("-----");
		//strm->oMsg("Term no semafor :%u", mNoTermSmfCnt);
		//strm->oMsg("Term full TX buf:%u", mFullTxCnt);
		//strm->oMsg("RestartRxCnt    :%u", myConnection->mReStartCnt);

		strm->oClose();
	}
}

static void funShowHardware(OutStream *strm, const char *cmd, void *arg) {
	static int showCnt;
	if (strm->oOpen(colYELLOW)) {
		strm->oMsg("--- %u ----------", showCnt++);
		strm->oMsg("CFG0:%s", ST3Str(Hdw::getPinCfg0()));
		strm->oMsg("CFG1:%s", ST3Str(Hdw::getPinCfg1()));
		strm->oMsg("CFG2:%s", ST3Str(Hdw::getPinCfg2()));
		strm->oMsg("CFG3:%s", ST3Str(Hdw::getPinCfg3()));
		strm->oMsg("DUST_ON:%s", YN(Hdw::getDustSensorOn()));
		strm->oMsg("DUST_FLG:%s", ErrOk(Hdw::getDustSensorFlg()));
		strm->oMsg("HEATER_ON:%s", YN(Hdw::getHeaterOn()));
		strm->oMsg("HEATER_FLG:%s", ErrOk(Hdw::getHeaterFlg()));
		strm->oClose();
	}

}

static void funReboot(OutStream *strm, const char *cmd, void *arg) {
	strm->oMsgX(colRED, "*** R E S E T ***");
	reboot(1000);
}

static void funMenuConfig(OutStream *strm, const char *cmd, void *arg) {
	config->shell(strm, cmd);
}

extern const ShellItemFx menuTimeFx[];
static void funMenuTime(OutStream *strm, const char *cmd, void *arg) {
	execMenuCmd(strm, menuTimeFx, cmd, arg, "Time Menu");
}

extern const ShellItemFx menuEthFx[];
static void funMenuEth(OutStream *strm, const char *cmd, void *arg) {
	execMenuCmd(strm, menuEthFx, cmd, arg, "Ethernet Menu");
}

extern const ShellItemFx menuNetFx[];
static void funMenuIP(OutStream *strm, const char *cmd, void *arg) {
	execMenuCmd(strm, menuNetFx, cmd, arg, "Net Menu");
}

static void funMenuBG96(OutStream *strm, const char *cmd, void *arg) {
	bg96->shell(strm, cmd);
	//flgSendAny = true;
}
static void funMenuI2C(OutStream *strm, const char *cmd, void *arg) {
	i2cBus1->shell(strm, cmd);
}
static void funMenuDust(OutStream *strm, const char *cmd, void *arg) {
	if (config->data.P.dustInpType == dust_Intern) {
		dustInternSensor->shell(strm, cmd);
	} else {
		//dustExternSensor->shell(strm, cmd);
	}
}
static void funMenuMdb1(OutStream *strm, const char *cmd, void *arg) {
	mdbMaster_1->shell(strm, cmd);
}
static void funMenuMdb2(OutStream *strm, const char *cmd, void *arg) {
	mdbMaster_2->shell(strm, cmd);
}
static void funMenuLedMatrix(OutStream *strm, const char *cmd, void *arg) {
	if (ledMatrix != NULL) {
		ledMatrix->shell(strm, cmd);
	} else {
		strm->oMsgX(colRED, "Obsluga LedMatrix wylaczona");
	}
}

static void funShowTask(OutStream *strm, const char *cmd, void *arg) {
	if (strm->oOpen(colWHITE)) {
		char *bigbuf;
		bigbuf = (char*) malloc(2000);
		if (bigbuf != NULL) {
			vTaskList(bigbuf);
			strm->oMsg("TXT_SIZE=%u (max %u) (p=%08X)", strlen(bigbuf), 2000, (int) bigbuf);
			strm->oMsg("Name\t\tState\tPrior.\tStackP\tNum");
			strm->oWr(bigbuf);
			free(bigbuf);
		} else
			strm->oMsg("NoFreeMem for Buffer");

		strm->oClose();
	}
}

static void funShowTaskEx(OutStream *strm, const char *cmd, void *arg) {
	TaskClassList::ShowList(strm);
}

extern uint8_t _end; /* Symbol defined in the linker script */
extern uint8_t _estack; /* Symbol defined in the linker script */
extern uint32_t _Min_Stack_Size; /* Symbol defined in the linker script */
void *mem_try = NULL;

static void funShowMemUsage(OutStream *strm, const char *cmd, void *arg) {
	if (strm->oOpen(colWHITE)) {
		if (mem_try == NULL)
			mem_try = malloc(4);

		const uint32_t stack_start = (uint32_t) &_end;
		const uint32_t stack_limit = (uint32_t) &_estack - (uint32_t) &_Min_Stack_Size;

		strm->oMsg("heap_begin= %p", stack_start);
		strm->oMsg("heap_end= %p", stack_limit);
		strm->oMsg("heap_curr = %p", mem_try);

		int size = stack_limit - stack_start;
		int used = (int) mem_try - stack_start;
		int freem = size - used;
		int stack_sz = (uint32_t) &_Min_Stack_Size;

		strm->oMsg("heap_size = %u (%p)", size, size);
		strm->oMsg("heap_used = %u (%p)", used, used);
		strm->oMsg("heap_free = %u (%p)", freem, freem);
		strm->oMsg("stack_size = %u (%p)", stack_sz, stack_sz);

		strm->oClose();
	}

}

static void funGlobData(OutStream *strm, const char *cmd, void *arg) {
	GlobData::show(strm);
}
static void funJGlobData(OutStream *strm, const char *cmd, void *arg) {
	GlobData::showJson(strm);
}

static void funJGlobDef(OutStream *strm, const char *cmd, void *arg) {
	GlobData::showDef(strm);
}

static void funLcdScr(OutStream *strm, const char *cmd, void *arg) {
	int v;
	Token::getAsInt(&cmd, &v);
	setLcdScrNr(v);
}
static void funLcdTime(OutStream *strm, const char *cmd, void *arg) {
	int v;
	Token::getAsInt(&cmd, &v);
	setLcdTime(v);
}
static void funNTC(OutStream *strm, const char *cmd, void *arg) {
	NTC::StartMeasure();
	if (NTC::WaitForMeasEnd(500)) {
		strm->oMsgX(colWHITE, "NTC: U=%.3f[V] R=%.2f[kom] temp=%.1f[deg]", NTC::nap, NTC::rez / 1000, NTC::temp);
	}
}

const ShellItemFx mainMenuFx[] = { //
		{ "s", "[F2] status urządzenia", funShowState }, //
				{ "h", "[F3] stan hardware", funShowHardware }, //
				{ "reboot", "reboot STM", funReboot }, //
				{ "cfg", ">> menu konfiguracji", funMenuConfig }, //
				{ "time", ">> menu czasu", funMenuTime }, //
				{ "eth", ">> menu etherneta", funMenuEth }, //
				{ "net", ">> menu tcp/ip", funMenuIP }, //
				{ "bg", ">> menu BG96", funMenuBG96 }, //
				{ "iic", ">> menu układów i2c", funMenuI2C }, //
				{ "dust", ">> menu czujnika pyłów", funMenuDust }, //
				{ "mdb1", ">> menu modbus master X7", funMenuMdb1 }, //
				{ "mdb2", ">> menu modbus master X6", funMenuMdb2 }, //
				{ "matrix", ">> menu LedMatrix", funMenuLedMatrix }, //
				{ "ps", "lista wątków", funShowTask }, //
				{ "psx", "lista tasków", funShowTaskEx }, //
				{ "mem", "informacja o pamięci", funShowMemUsage }, //
				{ "glob", "dane globalne", funGlobData }, //
				{ "jglob", "dane globalne w postaci json", funJGlobData }, //
				{ "globdef", "definicja danych globalnych", funJGlobDef }, //
				{ "lcd_scr", "ustawienie numer wyświetlanego ekranu", funLcdScr }, //
				{ "lcd_time", "ustawienie czasu przełaczania ekranów na lcd", funLcdTime }, //
				{ "ntc", "pokaż temperaturę z NTC", funNTC }, //

				{ NULL, NULL } };

//--------EthMenu-----------------------------------------------------------------

static void funEthStatus(OutStream *strm, const char *cmd, void *arg) {

}
static void funEthResetPhy(OutStream *strm, const char *cmd, void *arg) {
	strm->oMsgX(colWHITE, "PHY Reset");
	Hdw::phyReset(1);
	osDelay(50);
	Hdw::phyReset(0);
}
static void funEthPhyOnOff(OutStream *strm, const char *cmd, void *arg) {
	int val;
	if (Token::getAsInt(&cmd, &val)) {
		strm->oMsgX(colWHITE, "PHY power = %d", val);
		Hdw::phyPower(val != 0);
	}
}

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

static void funEthShowPhy(OutStream *strm, const char *cmd, void *arg) {
	uint16_t adrTab[30];
	uint16_t valTab[30];
	int k = 0;
	while (phyRegTab[k].name != NULL) {
		adrTab[k] = phyRegTab[k].adr;
		k++;
	}
	adrTab[k] = 0xFFFF;
	ethGetPhyReg(adrTab, valTab);
	if (strm->oOpen(colWHITE)) {
		int k = 0;
		while (phyRegTab[k].name != NULL) {
			strm->oMsg("%2u. %04X %s", phyRegTab[k].adr, valTab[k], phyRegTab[k].name);
			k++;
		}
		strm->oClose();
	}
}

const ShellItemFx menuEthFx[] = { //
		{ "s", "status etherneta", funEthStatus }, //
				{ "reset", "impuls reset do PHY", funEthResetPhy }, //
				{ "pwr", "=0 (wyłącz), =1(załącz) zasilanie PHY", funEthPhyOnOff }, //
				{ "reg", "pokaż rejestry PHY", funEthShowPhy }, //
				{ NULL, NULL } };

//--------TimeMenu-----------------------------------------------------------------

static void funShowTime(OutStream *strm, const char *cmd, void *arg) {
	TDATE tm;
	char buf[30];
	if (Rtc::ReadTime(&tm)) {
		TimeTools::DtTmStrZZ(buf, &tm);
		strm->oMsgX(colGREEN, buf);
	} else {
		strm->oMsgX(colRED, "Błąd pobrania czasu");
	}
}

static void funShowTimeInfo(OutStream *strm, const char *cmd, void *arg) {
	TDATE tm;
	char buf[30];
	if (Rtc::ReadTime(&tm)) {
		TimeTools::DtTmStrZZ(buf, &tm);
		strm->oMsgX(colGREEN, "Czas:%s", buf);
	} else {
		strm->oMsgX(colRED, "Błąd pobrania czasu");
	}
	TimeTools::DtTmStrZZ(buf, &config->data.R.rtcSetUpTime);
	strm->oMsgX(colGREEN, "Czas ustawienia czasu: %s", buf);
	strm->oMsgX(colGREEN, "Zródło ustawienia czasu: %s", getTmSrcName(config->data.R.rtcSetUpTime.timeSource));

}
static void funSetTime(OutStream *strm, const char *cmd, void *arg) {
	TDATE tm;
	if (TimeTools::parseTime(&cmd, &tm)) {
		if (Rtc::SetTime(&tm)) {
			Rtc::ReadTime(&tm);
			tm.timeSource = tmSrcNTP;
			config->data.R.rtcSetUpTime = tm;
			config->saveRtc();
			strm->oMsgX(colGREEN, "Ok");
		} else
			strm->oMsgX(colRED, "Set time error");
	} else
		strm->oMsgX(colRED, "Time format error");
}

static void funSetDate(OutStream *strm, const char *cmd, void *arg) {
	TDATE tm;
	if (TimeTools::parseDate(&cmd, &tm)) {
		if (Rtc::SetDate(&tm)) {
			Rtc::ReadTime(&tm);
			tm.timeSource = tmSrcNTP;
			config->data.R.rtcSetUpTime = tm;
			config->saveRtc();
			strm->oMsgX(colGREEN, "Ok");
		} else
			strm->oMsgX(colRED, "Set date error");
	} else
		strm->oMsgX(colRED, "Date format error");
}

static void funInitRtc(OutStream *strm, const char *cmd, void *arg) {
	Rtc::Init();
	strm->oMsgX(colWHITE, "RtcInitStatus   :%s", HAL_getErrStr(Rtc::mRtcStatus));
}

const ShellItemFx menuTimeFx[] = { //
		{ "s", "pokaż czas", funShowTime }, //
				{ "i", "pokaż informacje", funShowTimeInfo }, //
				{ "settime", "ustaw czas - gg:mm:ss", funSetTime }, //
				{ "setdate", "ustaw date - rrrr.mm.dd", funSetDate }, //
				{ "init", "init RTC", funInitRtc }, //
				{ NULL, NULL, NULL } };

//--------NetMenu-----------------------------------------------------------------

static void funIPStatus(OutStream *strm, const char *cmd, void *arg) {
	NetState netState;
	char txt[20];

	getNetIfState(&netState);
	if (strm->oOpen(colWHITE)) {
		strm->oMsg("LinkUp:%s", YN(netState.LinkUp));
		strm->oMsg("Dhcp:%s", OnOff(netState.DhcpOn));
		strm->oMsg("DhcpRdy:%s", YN(netState.DhcpRdy));
		if (netState.ipValid) {
			ipaddr_ntoa_r(&netState.CurrIP, txt, sizeof(txt));
			strm->oMsg("IP:%s", txt);
			ipaddr_ntoa_r(&netState.CurrMask, txt, sizeof(txt));
			strm->oMsg("Mask:%s", txt);
			ipaddr_ntoa_r(&netState.CurrGate, txt, sizeof(txt));
			strm->oMsg("GateWay:%s", txt);
		}
		const ip_addr_t *pdns1 = dns_getserver(0);
		ipaddr_ntoa_r(pdns1, txt, sizeof(txt));
		strm->oMsg("DNS_1:%s", txt);
		const ip_addr_t *pdns2 = dns_getserver(1);
		ipaddr_ntoa_r(pdns2, txt, sizeof(txt));
		strm->oMsg("DNS_2:%s", txt);

		strm->oClose();
	}
}

static void funIPRestart(OutStream *strm, const char *cmd, void *arg) {
	strm->oMsgX(colWHITE, "Network reconfig");
	reconfigNet();
}

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

static void funIPUseDns(OutStream *strm, const char *cmd, void *arg) {
	Token::trim(&cmd);
	int err = dns_gethostbyname(cmd, &globAddr, &dns_found_cb, NULL);
	switch (err) {
	case ERR_OK: //
	{
		char txt[20];
		ipaddr_ntoa_r(&globAddr, txt, sizeof(txt));
		strm->oMsgX(colGREEN, "IME %s -> %s", cmd, txt);
	}
		break;
	case ERR_INPROGRESS:
		break;
	default:
		strm->oMsgX(colRED, "Error :%d", err);
	}
}

int Ping(ip4_addr_t addr, int length);
static void funIPPing(OutStream *strm, const char *cmd, void *arg) {
	Token::trim(&cmd);

	ip4_addr_t addr;
	if (ipaddr_aton(cmd, &addr)) {
		if (strm->oOpen(colWHITE)) {
			Ping(addr, 32);
			strm->oClose();
		}
	}
}

const ShellItemFx menuNetFx[] = { //
		{ "s", "pokaż stan", funIPStatus }, //
				{ "restart", "rekonfiguruj net", funIPRestart }, //
				{ "getip", "użyj DNS", funIPUseDns }, //
				{ "ping", "ping", funIPPing }, //
				{ NULL, NULL } };

//-----------------------------------------------------------------------------------

/**
 * return number of milliseconds since boot time (or any other reference)
 */
#define sys_start_tickcount()  HAL_GetTick()

/**
 * return number of milliseconds since "ref"
 */
u32_t sys_stop_tickcount(u32_t ref) {
	return HAL_GetTick() - ref;

}

#define PING_TIMEOUT 1000
/*----------------------------------------------------------------------------*/
int Ping(ip4_addr_t addr, int length)
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
	OutStream *strm = getOutStream();

	char txt[20];
	ipaddr_ntoa_r(&addr, txt, sizeof(txt));

	strm->oMsg("Ping %s : bytes=%i\n", txt, length);

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

				} while ((err > 0) && ((pechoreply->id != pecho->id) || (pechoreply->seqno != pecho->seqno)) && (sys_stop_tickcount(ulTickCount) <= (u32_t)timeout));
				ulTickCount = sys_stop_tickcount(ulTickCount);

				/* Si le "ping" est bien reçu... */
				if (err > 0) { /* Affichage dans la console */
					ipaddr_ntoa_r(&addr, txt, sizeof(txt));

					strm->oMsg("Ping Reply from %s : bytes=%i, delay=%lums\n", txt, (err - sizeof(struct icmp_echo_hdr))/*length*/, ulTickCount);
					iResult = 0;
				}
				/* Si on a eu une "non-réception"... */
				else { /* Affichage dans la console */
					strm->oMsg("Ping Request timed out...\n");
					iResult = ETIMEDOUT;
				}
			} else {
				iResult = errno;
				strm->oMsg("Ping sendto=%i\n", iResult);
			}
		}

		closesocket(hSocket);
	} else {
		iResult = errno;
		strm->oMsg("Ping socket=%d\n", iResult);
	}

	free(pecho);

	return iResult;
}
