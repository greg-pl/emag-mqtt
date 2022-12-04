/*
 * UMain.cpp
 *
 *  Created on: Dec 5, 2020
 *      Author: Grzegorz
 */

#include "math.h"

#include "cmsis_os.h"
#include "lwip.h"
#include "usb_device.h"
#include "dns.h"

#include "UMain.h"
#include "utils.h"
#include "TaskClass.h"
#include "Shell.h"
#include "Config.h"
#include "Bg96Driver.h"
#include <I2cDev.h>
#include <IOStream.h>
#include "DustPMSA.h"
#include "SPS30.h"
#include "ssd1306/ssd1306.h"
#include "GlobData.h"
#include "MdbMasterTask.h"
#include "LedMatrix.h"

extern IWDG_HandleTypeDef hiwdg;

EventGroupHandle_t sysEvents;

ShellTask *shellTask;
MdbMasterNoiseTask *mdbMaster_1;
MdbMasterGasTask *mdbMaster_2;
LedMatrix *ledMatrix;

Config *config;
Bg96Driver *bg96;
SHT35DevPub *sht35;
Bmp338DevPub *bmp338;
DustSensorBase *dustInternSensor;
MdbMasterDustTask *dustExternSensor;

SSD1306Dev *lcd;
VerInfo mSoftVer;

#define SIGN_NO_INIT1  0x34568923
#define SIGN_NO_INIT2  0xAAFFEECC

SEC_NOINIT NIR nir;

//-------------------------------------------------------------------------------------------------------------------------
// LABEL
//-------------------------------------------------------------------------------------------------------------------------
#define SEC_LABEL   __attribute__ ((section (".label")))
SEC_LABEL char DevLabel[] = "TTT             "
		"                "
		"                "
		"                "
		"****************"
		"*    AIR-PRO   *"
		"*              *"
		"****************";

extern "C" OutStream* getOutStream() {
	return shellTask;
}

//-------------------------------------------------------------------------------------------------------------------------
// NTC
//-------------------------------------------------------------------------------------------------------------------------

short int NTC::mBuffer[ADC1_BUFFER_LEN];
uint32_t NTC::mStartTick;
uint32_t NTC::mDeltaTick;
float NTC::nap;
float NTC::rez;
float NTC::temp;
volatile bool NTC::mMeasDone;
volatile bool NTC::mNewMeas;

volatile int NTC::mError;

extern ADC_HandleTypeDef hadc1;

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc) {
	NTC::OnConvError(hadc);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	NTC::OnConvEnd();
}

void NTC::OnConvError(ADC_HandleTypeDef *hadc) {
	mError = hadc->ErrorCode;
	mDeltaTick = HAL_GetTick() - mStartTick;
	mMeasDone = true;
}

void NTC::OnConvEnd() {
	mError = HAL_OK;
	mDeltaTick = HAL_GetTick() - mStartTick;
	nap = getNtcNap();
	rez = liczRez(nap);
	temp = liczTemp(rez);
	mMeasDone = true;
	mNewMeas = true;
}

bool NTC::isNewMeas() {
	bool q = mNewMeas;
	mNewMeas = false;
	return q;
}

void NTC::StartMeasure(void) {
	memset(mBuffer, 0, sizeof(mBuffer));
	mStartTick = HAL_GetTick();
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) (dword) &mBuffer, sizeof(mBuffer) / sizeof(short int));
	HAL_ADC_Start(&hadc1);
	mMeasDone = false;
}

bool NTC::WaitForMeasEnd(int maxTime) {
	uint32_t tt = HAL_GetTick();

	while (HAL_GetTick() - tt < (uint32_t) maxTime) {
		if (mMeasDone) {
			return true;
		}
		osDelay(50);
	}
	return false;
}

float NTC::getNtcNap() {
	int sum = 0;
	for (int i = 0; i < ADC1_BUFFER_LEN; i++) {
		sum += mBuffer[i];
	}
	float f = (float) sum / ADC1_BUFFER_LEN;
	f /= 4096;
	f *= 3.3;
	return f;
}

float NTC::liczRez(float u) {
	return 100000 * u / (3.3 - u);
}

//rt - rezystancja w OM
//wyjście: tempertura w stop.Celsjusza
float NTC::liczTemp(float rt) {
	const double r0 = 100000;
	const double a1 = 0.003354016;
	const double b1 = 0.000248277;
	const double c1 = 1.8399E-06;
	const double d1 = -2.61671E-08;

	double m = log(rt / r0);
	double m2 = m * m;
	double m3 = m2 * m;
	double v = a1 + b1 * m + c1 * m2 + d1 * m3;
	double w = 1 / v;
	double t = w - 273.15;
	return t;
}

//-------------------------------------------------------------------------------------------------------------------------
// DefaultTask
//-------------------------------------------------------------------------------------------------------------------------

class DefaultTask: public TaskClass, public SignaledClass {
private:
	enum {
		SIGNAL_TICK = 0x01, //
	};

	void doNetStatusChg();
	enum {
		scrNoDef = 0, //
		scrWelcome, //
		scrState, //
		scrMeasure, //
		scrIP, //
		scr__MAX
	};

	struct {
		int cfgScrNr;
		int cfgSwitchTime;

		int scrNr;
		uint32_t redrawTick; // czas ostatniego odrysowania
		uint32_t scrStartTick; // czas włączenia danego okienka
		bool showSms;
	} lcdState;

	struct {
		uint32_t lastRunTick;
		bool waitNtcMeasure;
		bool on;
		bool regTempOut;
	} heater;
	void showIpScr();
	void showMeasureScr();
	void showStateScr();
	void lcdTick();
	void led3ColTick();
	bool getHdwError();
	void heaterTick();

protected:

	virtual void ThreadFunc();
public:
	DefaultTask();
	virtual void setSignal();
	void setLcdScrNr(int nr);
	void setLcdTime(int time);

};

DefaultTask::DefaultTask() :
		TaskClass::TaskClass("AIR-PRO", osPriorityNormal, 1024) {
	heater.on = false;
}

struct {
	volatile bool flag;
	uint32_t tick;
	uint32_t time;
} rebootRec = { 0 };

void DefaultTask::setSignal() {
	osSignalSet(getThreadId(), SIGNAL_TICK);
}

void setStatusNetIf(netif_status_callback_fn status_callback);

volatile uint8_t mNetIfStatusChg;

void NetIfStatusCallBack(struct netif *netif) {
	mNetIfStatusChg = 1;
}

void ethernetif_notify_conn_changed(struct netif *netif) {
	mNetIfStatusChg = 1;
}

void DefaultTask::doNetStatusChg() {
	shellTask->oMsgX(colYELLOW, "Net interface status changed.");
	NetState netState;
	getNetIfState(&netState);

	if (netState.DhcpOn) {
		if (!netState.DhcpRdy) {
			if (netState.LinkUp) {
				setDynamicIP();
			} else {
				clrNetIfAddr();
			}
		}
	}
}

void DefaultTask::setLcdScrNr(int nr) {
	if (nr >= scrWelcome && nr <= scr__MAX) {
		lcdState.cfgScrNr = nr;
		lcdState.scrNr = nr;
		lcdState.cfgSwitchTime = 0;
	}
}
void DefaultTask::setLcdTime(int time) {
	if (time >= 0) {
		lcdState.cfgSwitchTime = time;
	}
}

void DefaultTask::showIpScr() {
	NetState netState;
	char txt[40];

	getNetIfState(&netState);

	lcd->prn("%u.LinkUp=%s\n", lcdState.scrNr, YN(netState.LinkUp));
	lcd->prn("Dhcp=%s Rdy%s\n", OnOff(netState.DhcpOn), YN(netState.DhcpRdy));

	if (netState.ipValid) {
		ipaddr_ntoa_r(&netState.CurrIP, txt, sizeof(txt));
		lcd->prn("IP:%s\n", txt);
		ipaddr_ntoa_r(&netState.CurrMask, txt, sizeof(txt));
		lcd->prn("MS:%s\n", txt);
		ipaddr_ntoa_r(&netState.CurrGate, txt, sizeof(txt));
		lcd->prn("GW:%s\n", txt);
	}
	const ip_addr_t *pdns1 = dns_getserver(0);
	ipaddr_ntoa_r(pdns1, txt, sizeof(txt));
	lcd->prn("D1:%s\n", txt);

	const ip_addr_t *pdns2 = dns_getserver(1);
	ipaddr_ntoa_r(pdns2, txt, sizeof(txt));
	lcd->prn("D2:%s\n", txt);
}

void DefaultTask::showStateScr() {
	char buf[20];
	lcd->prn("%u.BG96\n", lcdState.scrNr);
	lcd->prn("Ph:%s\n", getPhaseName(bg96->getPhase()));
	lcd->prn("IM:%s\n", bg96->bgParam.imei);
	bool mqttOk = bg96->isMqttSendingOk();
	lcd->prn("MQTT=%s\n", ErrOk(!mqttOk));
	if (mqttOk) {
		lcd->prn("MsgId=%u\n", bg96->state.mqtt.mSendMsgID);
		lcd->prn("Tm=%s\n", TimeTools::TimeLongStr(buf, HAL_GetTick() - bg96->state.mqtt.sentTick));
	}

}

void DefaultTask::showMeasureScr() {
	float tab[SENSOR_CNT];
	GlobData::FillMeas(tab);

	lcd->prn("%u.Temp=%.2f[deg]\n", lcdState.scrNr, tab[ssTEMPERATURE]);
	lcd->prn("Humi =%.1f[%%]\n", tab[ssHUMIDITY]);
	lcd->prn("Press=%.2f[kPa]\n", tab[ssPRESSURE]);
	lcd->prn("PM1.0=%.1f[ug/m3]\n", tab[ssPM1_0]);
	lcd->prn("PM2.5=%.1f[ug/m3]\n", tab[ssPM2_5]);
	lcd->prn("PM10 =%.1f[ug/m3]\n", tab[ssPM10]);
}

void DefaultTask::lcdTick() {

	if (HAL_GetTick() - bg96->state.sms.reciveTick < 5000) {
		if (!lcdState.showSms) {
			lcd->clear();
			lcd->setFont(SSD1306Dev::fn16x26);
			lcd->setCursor(40, 0);
			lcd->wrStr("SMS");

			lcd->setCursor(0, 28);
			lcd->setFont(SSD1306Dev::fn7x10);
			lcd->wrStr(bg96->state.sms.nrTel);
			lcd->wrStr("\n");
			if (strlen(bg96->state.sms.msg) < 8)
				lcd->setFont(SSD1306Dev::fn11x18);

			lcd->wrStr(bg96->state.sms.msg);
			lcd->updateScr();
			lcdState.showSms = true;
		}
		return;
	}
	lcdState.showSms = false;

	uint32_t tt = HAL_GetTick();
	if (lcdState.cfgSwitchTime > 0) {
		if (tt - lcdState.scrStartTick > (uint32_t) (1000 * lcdState.cfgSwitchTime)) {
			lcdState.scrStartTick = tt;
			if (++lcdState.scrNr == scr__MAX) {
				lcdState.scrNr = scrWelcome;
			}
		}
	}
	if (tt - lcdState.redrawTick > 250) {
		lcdState.redrawTick = tt;

		lcd->clear();
		lcd->setFont(SSD1306Dev::fn7x10);

		switch (lcdState.scrNr) {
		case scrWelcome:
			lcd->setFont(SSD1306Dev::fn16x26);
			lcd->wrStr("AIR-PRO\n");
			lcd->setFont(SSD1306Dev::fn7x10);
			lcd->wrStr("\nMQTT-BG96");
			break;
		case scrState:
			showStateScr();
			break;
		case scrMeasure:
			showMeasureScr();
			break;
		case scrIP:
			showIpScr();
			break;

		};
		lcd->updateScr();
	}
}

void DefaultTask::heaterTick() {
	float temp;
	float humidity;
	bool measOk = false;

	if (config->data.R.exDev.heater.runInternal) {
		uint32_t tt = HAL_GetTick();
		if (tt - heater.lastRunTick > 2000) {
			heater.lastRunTick = tt;
			heater.waitNtcMeasure = true;
			NTC::StartMeasure();
		}
		if (heater.waitNtcMeasure) {
			if (NTC::isNewMeas()) {
				heater.waitNtcMeasure = false;

				sht35->getData(&temp, &humidity);
				if (config->data.R.exDev.heater.useNTCtemp)
					temp = NTC::temp;
				measOk = true;

				bool qr = heater.regTempOut;
				uint32_t dd = 0;
				if (config->data.R.exDev.heater2.humidityEnab) {
					dd |= 0x000001;
					if (!qr) {
						dd |= 0x000002;
						if ((temp < config->data.R.exDev.heater.tempON) || (humidity > config->data.R.exDev.heater.humidityON)) {
							qr = true;
							dd |= 0x000004;
						}
					} else {
						dd |= 0x000010;
						if ((temp > config->data.R.exDev.heater.tempOFF) && (humidity < config->data.R.exDev.heater.humidityOFF)) {
							qr = false;
							dd |= 0x000020;
						}
					}
				} else {
					dd |= 0x000100;
					if (!qr) {
						dd |= 0x000200;
						if (temp < config->data.R.exDev.heater.tempON) {
							qr = true;
							dd |= 0x000400;
						}
					} else {
						dd |= 0x000800;
						if (temp > config->data.R.exDev.heater.tempOFF) {
							qr = false;
							dd |= 0x001000;
						}
					}

				}

				if (config->data.R.exDev.heater.showMsg >= 2) {
					shellTask->oMsgX(colYELLOW, "HEATER-%s, REG-%s QR-%s T=%.1f[*C] H=%.0f[%%] useNTC=%u tempOFF=%.1f[*C] DD=0x%08X", OnOff(heater.on), OnOff(heater.regTempOut), OnOff(qr), //
					temp, humidity, config->data.R.exDev.heater.useNTCtemp, config->data.R.exDev.heater.tempOFF, dd);
				}

				heater.regTempOut = qr;

			}
		}
	} else {
		heater.regTempOut = false;
	}

	// załączenie od włożonego Jumpera na CFG3

	bool qw = heater.regTempOut || (Hdw::getPinCfg3() == posGND);

	if (qw != heater.on) {
		heater.on = qw;
		Hdw::heaterOn(heater.on);

		if (config->data.R.exDev.heater.showMsg >= 1) {
			char buf[20];
			TDATE tm;
			if (Rtc::ReadOnlyTime(&tm))
				TimeTools::TimeStrZZ(buf, &tm);
			else
				strcpy(buf, "??:??:??,??");
			if (measOk) {
				shellTask->oMsgX(colYELLOW, "%s HEATER-%s, T=%.1f[st.C] H=%.0f[%%]", buf, OnOff(qw), temp, humidity);
			} else {
				shellTask->oMsgX(colYELLOW, "%s HEATER-%s", buf, OnOff(qw));
			}
		}
	}
}

bool DefaultTask::getHdwError() {
	bool q = false;
	if (config->data.P.dustInpType == dust_Intern) {
		q |= dustInternSensor->isError();
	} else {
		q |= dustExternSensor->isError();
	}
	q |= I2c1Bus::isError();

	if (mdbMaster_1->isCfgNoiseOn()) {
		q |= mdbMaster_1->isError();
	}
	if (mdbMaster_2->isCfgAnyGas()) {
		q |= mdbMaster_2->isError();
	}
	return q;
}

void DefaultTask::led3ColTick() {
	static int ledPh = 0;
	static uint32_t ledTT = 0;

	if (HAL_GetTick() - ledTT > 250) {
		ledTT = HAL_GetTick();
		if (!config->data.R.rest.ledOff) {

			bool ledR = false;
			bool ledG = false;
			bool ledB = false;

			switch (ledPh) {
			case 0:
				break;
			case 1:
				ledR = getHdwError();
				break;
			case 2:
				ledB = bg96->isMqttSendingOk();
				break;
			case 3:
				ledG = !getHdwError() && !bg96->isMqttSendingOk();
				break;
			}

			ledPh = (ledPh + 1) & 0x03;
			Hdw::led3k(ledR, ledG, ledB);
		} else
			Hdw::led3k(0, 0, 0);
	}
}

void DefaultTask::ThreadFunc() {

	HAL_IWDG_Refresh(&hiwdg);
	__HAL_RCC_BKPSRAM_CLK_ENABLE();

	// MX_USB_DEVICE_Init();

	xEventGroupWaitBits(sysEvents, EVENT_TERM_RDY, false, false, 1000000);
	HAL_IWDG_Refresh(&hiwdg);

	I2c1Bus::BusInit();

	bmp338 = Bmp338DevPub::createDev(0xEC);
	sht35 = SHT35DevPub::createDev(0x88);
	lcd = new SSD1306Dev(0x78);
	HAL_IWDG_Refresh(&hiwdg);

	I2c1Bus::addDev(bmp338);
	I2c1Bus::addDev(sht35);
	I2c1Bus::addDev(lcd);

	config = new Config();
	config->Init(shellTask);

	Rtc::Init();
	HAL_IWDG_Refresh(&hiwdg);

	/* init code for LWIP */
	MX_LWIP_Init();
	HAL_IWDG_Refresh(&hiwdg);

	setStatusNetIf(&NetIfStatusCallBack);

	/* init code for USB_DEVICE */
	MX_USB_DEVICE_Init();

	bg96 = new Bg96Driver();
	bg96->Start();

	if (config->data.R.rest.ledMatrixRun) {
		ledMatrix = new LedMatrix(TUart::myUART4);
		ledMatrix->Init();
	}

	if (config->data.P.dustInpType == dust_Intern) {

		switch (config->data.P.dustSensorType) {
		case dustT_PMSA:  	//Chińczyk
			dustInternSensor = new DustPMSA(false);
			break;
		case dustT_PMS5003ST:
			dustInternSensor = new DustPMSA(true);
			break;
		case dustT_SPS30: //Siemens
			dustInternSensor = new SPS30();
			break;

		case dustT_HPMA: 	//Honeywell
		default:
			dustInternSensor = new DustSensorNull();
			break;
		}
		dustInternSensor->Init(this);
		dustInternSensor->StartMeas();
	} else {
		dustExternSensor = new MdbMasterDustTask(MdbMasterTask::MDB_3, TUart::myUART5);
		dustExternSensor->Start(9600, TUart::parityEVEN);
		dustExternSensor->setPower(true);
	}
	HAL_IWDG_Refresh(&hiwdg);

	xEventGroupSetBits(sysEvents, EVENT_CREATE_DEVICES);

	memset(&lcdState, 0, sizeof(lcdState));
	memset(&heater, 0, sizeof(heater));


	lcdState.cfgSwitchTime = 2000;
	lcdState.cfgScrNr = 1;
	lcdState.scrNr = lcdState.cfgScrNr;

	while (1) {
		osSignalWait(SIGNAL_TICK, 50);
		imAlive();

		if (rebootRec.flag) {
			if (HAL_GetTick() - rebootRec.tick > rebootRec.time) {
				NVIC_SystemReset();
			}
		}

		led3ColTick();

		if (mNetIfStatusChg) {
			mNetIfStatusChg = 0;
			doNetStatusChg();

		}
		I2c1Bus::tick();
		if (config->data.P.dustInpType == dust_Intern) {
			dustInternSensor->tick();
		}
		lcdTick();
		heaterTick();
		if (ledMatrix != NULL)
			ledMatrix->tick();

	}
}

//-------------------------------------------------------------------------------------------------------------------------
// WdgTask
//-------------------------------------------------------------------------------------------------------------------------
class WdgTask: public TaskClass {
private:
	enum {
		SIGNAL_TICK = 0x01, //
	};
protected:
	virtual void ThreadFunc();
public:
	WdgTask();
};

WdgTask::WdgTask() :
		TaskClass::TaskClass("WDG", osPriorityHigh, 256) {

}

void WdgTask::ThreadFunc() {

	HAL_IWDG_Refresh(&hiwdg);
	int upCnt = 0;
	while (true) {
		osSignalWait(SIGNAL_TICK, 1000);
		imAlive();
		TaskClassList::every1sek();

		TaskClass *deadTask = TaskClassList::isTasksAlive();
		if (deadTask == NULL) {
			HAL_IWDG_Refresh(&hiwdg);
			upCnt = 0;
		} else {
			shellTask->oMsgX(colRED, "DEAD_TASK:%s", deadTask->getThreadName());
			if (upCnt++ < 5) {
				HAL_IWDG_Refresh(&hiwdg);
			}
		}
	}
}

//-------------------------------------------------------------------------------------------------------------------------
// USB malloc
//-------------------------------------------------------------------------------------------------------------------------
char usb_buf[1024];
int usb_malloc_cnt;
extern "C" void* USBD_malloc(int size) {
	usb_malloc_cnt++;
	return usb_buf;
}
extern "C" void usb_free(void *ptr) {

}

//-------------------------------------------------------------------------------------------------------------------------
// UMain
//-------------------------------------------------------------------------------------------------------------------------
extern "C" void callResCubeMX();

DefaultTask *defaultTask;
WdgTask *wdgTask;

void setLcdScrNr(int nr) {
	defaultTask->setLcdScrNr(nr);
}
void setLcdTime(int time) {
	defaultTask->setLcdTime(time);
}

extern int _snoinit;
extern int _enoinit;
void initNIR() {
	if (nir.Sign1 != SIGN_NO_INIT1 || nir.Sign2 != SIGN_NO_INIT2) {
		void *adr = &_snoinit;
		int len = (int) &_enoinit - (int) &_snoinit;
		memset(adr, 0, len);
		nir.Sign1 = SIGN_NO_INIT1;
		nir.Sign2 = SIGN_NO_INIT2;
	}
}

void uMainCont() {

	initNIR();
	nir.startCnt++;

	callResCubeMX();

	nir.itmp1 = 0x1001;
	nir.itmp2 = 0x1002;
	nir.itmp3 = 0x1003;

	Hdw::heaterOn(0);
	Hdw::phyPower(1);
	Hdw::phyReset(0);
	if (!loadSoftVer(&mSoftVer, &DevLabel[16])) {
		mSoftVer.ver = 1;
		mSoftVer.rev = 138;
	}

	sysEvents = xEventGroupCreate();

	defaultTask = new DefaultTask();
	defaultTask->Start();

	wdgTask = new WdgTask();
	wdgTask->Start();

	shellTask = new ShellTask();
	shellTask->Start();

	mdbMaster_1 = new MdbMasterNoiseTask(MdbMasterTask::MDB_1, TUart::myUART1);
	mdbMaster_1->Start(9600, TUart::parityNONE);
	mdbMaster_1->setPower(true);

	mdbMaster_2 = new MdbMasterGasTask(MdbMasterTask::MDB_2, TUart::myUART3);
	mdbMaster_2->Start(9600, TUart::parityEVEN);
	mdbMaster_2->setPower(true);

	HAL_IWDG_Refresh(&hiwdg);

	osKernelStart();
	//tu nie powinno dojść
	while (1) {

	}

}

void getDevStatusAsTxt(char *buf, int max) {
	snprintf(buf, max, "OK"); //todo
}

void reboot(int tm) {
	rebootRec.tick = HAL_GetTick();
	rebootRec.time = tm;
	rebootRec.flag = true;
}

void shMsg(int color, const char *pFormat, ...) {
	shellTask->oMsgX((TermColor) color, pFormat);
}
unsigned char shMsgOpen(TermColor color) {
	return shellTask->oOpen((TermColor) color);
}
void shMsgClose() {
	shellTask->oClose();
}
void shMsgItem(const char *pFormat, ...) {
	shellTask->oMsg(pFormat);
}
