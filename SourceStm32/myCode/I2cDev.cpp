/*
 * I2cDev.cpp
 *
 *  Created on: 30 gru 2020
 *      Author: Grzegorz
 */

#include "string.h"
#include "stdio.h"
#include "math.h"

#include <I2cDev.h>
#include <main.h>
#include <Utils.h>
#include <cpx.h>
#include "Shell.h"
#include <ShellItem.h>

extern ShellTask *shellTask;

I2C_HandleTypeDef I2c1Bus::hi2c;
osMutexId I2c1Bus::mBusMutex;
int I2c1Bus::mDevCnt;
I2c1Dev *I2c1Bus::devTab[I2c1Bus::MAX_DEV_CNT];
int I2c1Bus::mLastMutexWho;
int I2c1Bus::mMutexWho;
int I2c1Bus::mBusRestartCnt = 0;

#define TIME_DT_RD      2000
#define TIME_DT_VALID   5000

#define FILTR_FACTOR  0.8

HAL_StatusTypeDef I2c1Bus::BusInit() {

	mDevCnt = 0;
	mBusRestartCnt = 0;
	for (int i = 0; i < MAX_DEV_CNT; i++) {
		devTab[i] = NULL;
	}

	osMutexDef(I2C1Dev);
	mBusMutex = osMutexCreate(osMutex(I2C1Dev));

	return InitHd();
}

HAL_StatusTypeDef I2c1Bus::_InitHd() {
	memset(&hi2c, 0, sizeof(hi2c));
	hi2c.Instance = I2C1;
	hi2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c.Init.ClockSpeed = 100000; //100kHz
	hi2c.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c.Init.OwnAddress1 = 0;
	hi2c.Init.OwnAddress2 = 0;
	hi2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	return HAL_I2C_Init(&hi2c);
}

HAL_StatusTypeDef I2c1Bus::InitHd() {

	HAL_StatusTypeDef st = HAL_BUSY;
	if (openMutex(1, 100)) {
		st = _InitHd();
		closeMutex();
	}
	return st;
}

HAL_StatusTypeDef I2c1Bus::BusUnlock() {
	if (openMutex(1, 100)) {
		SET_BIT(hi2c.Instance->CR1, I2C_CR1_SWRST);
		HAL_Delay(2);
		CLEAR_BIT(hi2c.Instance->CR1, I2C_CR1_SWRST);
		closeMutex();
	}
	return HAL_OK;
}

//PB6     ------> I2C1_SCL
//PB7     ------> I2C1_SDA

void I2c1Bus::setAsGpio() {
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	setGpioSDA(GPIO_PIN_SET);
	setGpioSCL(GPIO_PIN_SET);
}

void I2c1Bus::setGpioSDA(GPIO_PinState PinState) {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, PinState);
}
void I2c1Bus::setGpioSCL(GPIO_PinState PinState) {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, PinState);
}

bool I2c1Bus::getGpioSDA() {
	return (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET);
}

bool I2c1Bus::getGpioSCL() {
	return (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_SET);
}

void I2c1Bus::gpioSCLWave() {
	setAsGpio();
	for (int i = 0; i < 64; i++) {
		setGpioSCL(GPIO_PIN_RESET);
		osDelay(2);
		setGpioSCL(GPIO_PIN_SET);
		osDelay(2);
	}
}

HAL_StatusTypeDef I2c1Bus::BusRestart() {
	HAL_I2C_DeInit(&hi2c);
	HAL_StatusTypeDef st = InitHd();
	if (st == HAL_OK) {
		for (int i = 0; i < mDevCnt; i++) {
			devTab[i]->init();
		}
	}
	return st;
}

void I2c1Bus::ShowBusRegisters(OutStream *strm) {
	if (strm->oOpen(colWHITE)) {
		strm->oMsg("CR1:0x%08X", hi2c.Instance->CR1);
		strm->oMsg("CR2:0x%08X", hi2c.Instance->CR2);
		strm->oMsg("SR1:0x%08X", hi2c.Instance->SR1);
		strm->oMsg("SR2:0x%08X", hi2c.Instance->SR2);
		strm->oMsg("CCR  :0x%08X", hi2c.Instance->CCR);
		strm->oMsg("TRISE:0x%08X", hi2c.Instance->TRISE);
		strm->oMsg("FLTR :0x%08X", hi2c.Instance->FLTR);

		strm->oClose();
	}

}

bool I2c1Bus::openMutex(int who, int tm) {
	bool q = (osMutexWait(mBusMutex, tm) == osOK);
	if (q)
		mMutexWho = who;
	return q;
}
void I2c1Bus::closeMutex() {
	mLastMutexWho = mMutexWho;
	mMutexWho = 0;
	osMutexRelease(mBusMutex);
}

void I2c1Bus::swap(uint16_t *p) {
	uint8_t *pb = (uint8_t*) p;
	uint8_t b1 = pb[0];
	uint8_t b2 = pb[1];
	pb[0] = b2;
	pb[1] = b1;
}

uint16_t I2c1Bus::swapD(uint16_t d) {
	return (d >> 8) | (d << 8);
}

void I2c1Bus::putSwap(uint8_t *p, uint16_t d) {
	p[0] = d >> 8;
	p[1] = d & 0xff;
}

HAL_StatusTypeDef I2c1Bus::checkDev(uint8_t dev_addr) {
	return HAL_I2C_IsDeviceReady(&hi2c, dev_addr, 3, 100);
}

HAL_StatusTypeDef I2c1Bus::readBytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t len, uint8_t *data) {

	return HAL_I2C_Mem_Read(&hi2c, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, 100);
}

HAL_StatusTypeDef I2c1Bus::readByte(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data) {
	return readBytes(dev_addr, reg_addr, 1, data);
}

HAL_StatusTypeDef I2c1Bus::readWord(uint8_t dev_addr, uint8_t reg_addr, uint16_t *data) {
	uint16_t tmp;
	HAL_StatusTypeDef st = readBytes(dev_addr, reg_addr, 2, (uint8_t*) &tmp);
	*data = swapD(tmp);
	return st;
}

HAL_StatusTypeDef I2c1Bus::writeBytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t len, const uint8_t *data) {
	return HAL_I2C_Mem_Write(&hi2c, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, (uint8_t*) (int) data, len, 100);
}

HAL_StatusTypeDef I2c1Bus::writeByte(uint8_t dev_addr, uint8_t reg_addr, uint8_t data) {
	return writeBytes(dev_addr, reg_addr, 1, &data);
}

HAL_StatusTypeDef I2c1Bus::writeWord(uint8_t dev_addr, uint8_t reg_addr, uint16_t data) {
	data = swapD(data);
	return writeBytes(dev_addr, reg_addr, 2, (const uint8_t*) &data);
}

void I2c1Bus::addDev(I2c1Dev *dev) {
	if (mDevCnt < MAX_DEV_CNT) {
		devTab[mDevCnt] = dev;
		mDevCnt++;
	}
}

HAL_StatusTypeDef I2c1Bus::checkDevMtx(uint8_t dev_addr) {
	HAL_StatusTypeDef st = HAL_NO_SEMF;
	if (I2c1Bus::openMutex(2, 100)) {
		st = checkDev(dev_addr);
		I2c1Bus::closeMutex();
	}
	return st;
}

void I2c1Bus::ScanBus(OutStream *strm) {
	if (strm->oOpen(colWHITE)) {
		if (openMutex(3, 100)) {
			int devCnt = 0;
			for (int i = 0; i < 128; i++) {
				uint16_t dev_addr = 2 * i;
				if (HAL_I2C_IsDeviceReady(&hi2c, dev_addr, 2, 50) == HAL_OK) {
					strm->oMsg("Found adr=0x%02X", dev_addr);
					devCnt++;
				}
			}
			strm->oMsg("Found %u devices", devCnt);
			closeMutex();
		} else {
			strm->oMsg("I2c mutex error");
		}
		strm->oClose();
	}
}

void I2c1Bus::showState(OutStream *strm) {
	if (strm->oOpen(colWHITE)) {
		strm->oMsg("MutextWho=%d (Last=%d)", mMutexWho, mLastMutexWho);
		strm->oMsg("BusRestartCnt=%u", mBusRestartCnt);

		for (int i = 0; i < mDevCnt; i++) {
			devTab[i]->showState(strm);
		}
		strm->oClose();
	}
}
void I2c1Bus::showMeas(OutStream *strm) {
	if (strm->oOpen(colWHITE)) {
		for (int i = 0; i < mDevCnt; i++) {
			devTab[i]->showMeas(strm);
		}
		strm->oClose();
	}
}

bool I2c1Bus::rdBusyFlag() {
	return ((hi2c.Instance->SR2 & I2C_SR2_BUSY) != 0);
}
void I2c1Bus::tick() {
	if (rdBusyFlag()) {
		//zablokowanie magistrali- próba odblokowania
		if (openMutex(4, 100)) {
			setAsGpio();
			bool sdaBf = getGpioSDA();
			gpioSCLWave();
			bool sdaAf = getGpioSDA();
			bool busyAf = rdBusyFlag();
			closeMutex();
			BusRestart();
			shellTask->oMsgX(colRED, "I2CBus RESTART: sdaBf=%u, sdaAf=%u, busyAf=%u", sdaBf, sdaAf, busyAf);
			mBusRestartCnt++;
		}

	} else {
		for (int i = 0; i < mDevCnt; i++) {
			devTab[i]->tick();
		}
	}
}

bool I2c1Bus::isError() {
	bool q = 0;
	for (int i = 0; i < mDevCnt; i++) {
		q |= devTab[i]->isError();
	}
	return q;
}

void I2c1Bus::execFun(OutStream *strm, int idx) {
	if (strm->oOpen(colWHITE)) {
		for (int i = 0; i < mDevCnt; i++) {
			devTab[i]->execFun(strm, idx);
		}
		strm->oClose();
	}
}

const ShellItem menuI2C[] = { //
		{ "s", "stan" }, //
				{ "m", "pomiary" }, //
				{ "scan", "przeszukanie magistrali" }, //
				{ "restart", "restart magistrali" }, //
				{ "reg", "show iic registers" }, //
				{ "busUnlock", "odblokowanie magistrali i2c" }, //
				{ "rdGpio", "czytaj linie SDA,SCL jako GPIO" }, //
				{ "sclWave", "wygenerowanie fali na lnii SCL" }, //

				{ "fun", "wywołanie funkcji" }, //
				{ "SHT_heater", "wł/wył SHT35 heater" }, //
				{ "SHT_show_alert", "SHT35 pokaż allerty " }, //
				{ "SHT_set_alert", "SHT35 ustaw zestaw alertów, parametr: 1|2. " }, //
				{ "SHT_status", "SHT35 pokaż rejestr statusu" }, //
				{ "SHT_clear", "SHT35 czyść flagi alertów" }, //
				{ "SHT_reset", "SHT35 software reset" }, //
				{ "SHT_start", "SHT35 start pomiarów" }, //
				{ "SHT_sn", "SHT35 czytaj serial number" }, //
				{ "BMP_init", "inicjuj BMP338" }, //
				{ "SSD1306_init", "inicjuj SSD1306" }, //
				{ "SSD1306_test", "test screen" }, //

				{ NULL, NULL } };

void I2c1Bus::shell(OutStream *strm, const char *cmd) {
	char tok[20];
	int idx = -1;

	if (Token::get(&cmd, tok, sizeof(tok)))
		idx = findCmd(menuI2C, tok);
	switch (idx) {
	case 0: //s
		showState(strm);
		break;
	case 1: //m
		showMeas(strm);
		break;
	case 2: //scan
		ScanBus(strm);
		break;
	case 3: //restart
		BusRestart();
		break;
	case 4: //reg
		ShowBusRegisters(strm);
		break;
	case 5:	//busUnlock
		BusUnlock();
		strm->oMsgX(colWHITE, "Ok. Wykonaj restart i2c.");
		break;
	case 6:  //rdGpio
		if (strm->oOpen(colWHITE)) {
			setAsGpio();
			strm->oMsg("SDA:%u", getGpioSDA());
			strm->oMsg("SCL:%u", getGpioSCL());
			strm->oClose();
		}
		break;
	case 7: //sclWave
		gpioSCLWave();
		strm->oMsgX(colWHITE, "SCLWave. Wykonaj restart i2c.");
		break;

	case 8: { //fun{
		int nr;
		Token::getAsInt(&cmd, &nr);
		execFun(strm, nr);
	}
		break;
	case 9: { //heater
		int on;
		Token::getAsInt(&cmd, &on);
		if (on)
			execFun(strm, 3);
		else
			execFun(strm, 4);
	}
		break;
	case 10: //show_alert
		execFun(strm, 0);
		break;
	case 11: { //set_alert
		int nr;
		Token::getAsInt(&cmd, &nr);
		switch (nr) {
		case 1:
			execFun(strm, 1);
			break;
		case 2:
			execFun(strm, 2);
			break;
		default:
			strm->oMsg("parametr: 1|2");
			break;
		}
	}
		break;

	case 12:  //SHT_status
		execFun(strm, 5);
		break;
	case 13:  //SHT_clear
		execFun(strm, 6);
		break;
	case 14:  //SHT_reset
		execFun(strm, 7);
		break;
	case 15:  //SHT_start
		execFun(strm, 8);
		break;
	case 16:  //SHT_sn
		execFun(strm, 11);
		break;
	case 17:  //BMP_init
		execFun(strm, 20);
		break;
	case 18:  //SSD1306_init
		execFun(strm, 30);
		break;
	case 19: //SSD1306_test
		execFun(strm, 31);
		break;
	default:
		showHelp(strm, "I2C Menu", menuI2C);
		break;
	}

}

//-------------------------------------------------------------------------------------------------------------------------
// I2c1Dev
//-------------------------------------------------------------------------------------------------------------------------
void I2c1Dev::showDevExist(OutStream *strm) {
	HAL_StatusTypeDef st = I2c1Bus::checkDevMtx(getAdr());
	strm->oMsg("DevExist=%s", HAL_getErrStr(st));
}

//-------------------------------------------------------------------------------------------------------------------------
// SHT35Dev
//-------------------------------------------------------------------------------------------------------------------------
typedef enum {
	CMD_READ_SERIALNBR = 0x3780, // read serial number
	CMD_READ_STATUS = 0xF32D, // read status register
	CMD_CLEAR_STATUS = 0x3041, // clear status register
	CMD_HEATER_ENABLE = 0x306D, // enabled heater
	CMD_HEATER_DISABLE = 0x3066, // disable heater
	CMD_SOFT_RESET = 0x30A2, // soft reset
	CMD_BREAK = 0x3093, //break measure
	CMD_MEAS_CLOCKSTR_H = 0x2C06, // measurement: clock stretching, high repeatability
	CMD_MEAS_CLOCKSTR_M = 0x2C0D, // measurement: clock stretching, medium repeatability
	CMD_MEAS_CLOCKSTR_L = 0x2C10, // measurement: clock stretching, low repeatability
	CMD_MEAS_POLLING_H = 0x2400, // measurement: polling, high repeatability
	CMD_MEAS_POLLING_M = 0x240B, // measurement: polling, medium repeatability
	CMD_MEAS_POLLING_L = 0x2416, // measurement: polling, low repeatability
	CMD_MEAS_PERI_05_H = 0x2032, // measurement: periodic 0.5 mps, high repeatability
	CMD_MEAS_PERI_05_M = 0x2024, // measurement: periodic 0.5 mps, medium repeatability
	CMD_MEAS_PERI_05_L = 0x202F, // measurement: periodic 0.5 mps, low repeatability
	CMD_MEAS_PERI_1_H = 0x2130, // measurement: periodic 1 mps, high repeatability
	CMD_MEAS_PERI_1_M = 0x2126, // measurement: periodic 1 mps, medium repeatability
	CMD_MEAS_PERI_1_L = 0x212D, // measurement: periodic 1 mps, low repeatability
	CMD_MEAS_PERI_2_H = 0x2236, // measurement: periodic 2 mps, high repeatability
	CMD_MEAS_PERI_2_M = 0x2220, // measurement: periodic 2 mps, medium repeatability
	CMD_MEAS_PERI_2_L = 0x222B, // measurement: periodic 2 mps, low repeatability
	CMD_MEAS_PERI_4_H = 0x2334, // measurement: periodic 4 mps, high repeatability
	CMD_MEAS_PERI_4_M = 0x2322, // measurement: periodic 4 mps, medium repeatability
	CMD_MEAS_PERI_4_L = 0x2329, // measurement: periodic 4 mps, low repeatability
	CMD_MEAS_PERI_10_H = 0x2737, // measurement: periodic 10 mps, high repeatability
	CMD_MEAS_PERI_10_M = 0x2721, // measurement: periodic 10 mps, medium repeatability
	CMD_MEAS_PERI_10_L = 0x272A, // measurement: periodic 10 mps, low repeatability
	CMD_FETCH_DATA = 0xE000, // readout measurements for periodic mode
	CMD_R_AL_LIM_LS = 0xE102, // read alert limits, low set
	CMD_R_AL_LIM_LC = 0xE109, // read alert limits, low clear
	CMD_R_AL_LIM_HS = 0xE11F, // read alert limits, high set
	CMD_R_AL_LIM_HC = 0xE114, // read alert limits, high clear
	CMD_W_AL_LIM_HS = 0x611D, // write alert limits, high set
	CMD_W_AL_LIM_HC = 0x6116, // write alert limits, high clear
	CMD_W_AL_LIM_LC = 0x610B, // write alert limits, low clear
	CMD_W_AL_LIM_LS = 0x6100, // write alert limits, low set
	CMD_NO_SLEEP = 0x303E,
} etCommands;

typedef enum {
	REPEATAB_HIGH,   // high repeatability
	REPEATAB_MEDIUM, // medium repeatability
	REPEATAB_LOW,    // low repeatability
} etRepeatability;

// Measurement Mode
typedef enum {
	MODE_CLKSTRETCH, // clock stretching
	MODE_POLLING,    // polling
} etMode;

typedef enum {
	FREQUENCY_HZ5,  //  0.5 measurements per seconds
	FREQUENCY_1HZ,  //  1.0 measurements per seconds
	FREQUENCY_2HZ,  //  2.0 measurements per seconds
	FREQUENCY_4HZ,  //  4.0 measurements per seconds
	FREQUENCY_10HZ, // 10.0 measurements per seconds
} etFrequency;

typedef struct {
	float temperatureHighSet;
	float temperatureHighClear;
	float temperatureLowClear;
	float temperatureLowSet;
	float humidityHighSet;
	float humidityHighClear;
	float humidityLowClear;
	float humidityLowSet;
} AlertRec;

class SHT35Dev: public SHT35DevPub {
private:
	enum {
		POLYNOMIAL = 0x131, // P(x) = x^8 + x^5 + x^4 + 1 = 100110001
	};
	uint16_t rdSwap(uint8_t *p);
	HAL_StatusTypeDef ReadWordWithCrc(uint16_t cmdSHT, uint16_t *val);

	HAL_StatusTypeDef ReadSerialNumber(uint32_t *serialNumber);
	HAL_StatusTypeDef ReadStatus(uint16_t *status);
	uint8_t CalcCrc(uint8_t *data, uint8_t cnt);

	float CalcTemperature(uint16_t rawValue);
	float CalcHumidity(uint16_t rawValue);
	uint16_t CalcRawTemperature(float temperature);
	uint16_t CalcRawHumidity(float humidity);

	HAL_StatusTypeDef writeCmd(uint16_t cmd);
	HAL_StatusTypeDef EnableHeater(void);
	HAL_StatusTypeDef DisableHeater(void);
	HAL_StatusTypeDef ClearAllAlertFlags(void);
	HAL_StatusTypeDef SoftReset(void);
	HAL_StatusTypeDef SoftBreak(void);

	HAL_StatusTypeDef SendAlertData(uint16_t cmdSHT, float humidity, float temperature);
	HAL_StatusTypeDef ReadAlertData(uint16_t cmdSHT, float *humidity, float *temperature);
	HAL_StatusTypeDef setAlerts(AlertRec *rec);
	HAL_StatusTypeDef readAlerts(AlertRec *rec);
	HAL_StatusTypeDef StartPeriodicMeasurment(etRepeatability repeatability, etFrequency frequency);

	void showSerialNumer(OutStream *strm);
	void showStatus(OutStream *strm);
	void showMeasData(OutStream *strm);
	struct {
		float temp;
		float humi;
		uint32_t mReadTick;
	} meas;
	enum {
		MEAS_VALID = 1000, //1 sekunda
	};
	DtFilter filterTemp;
	DtFilter filterHumidity;
	uint32_t mLastRdDataTick;
	uint32_t mLastTryRdTick;
	HAL_StatusTypeDef readData();
protected:
	virtual void tick();
	virtual void init();
public:
	SHT35Dev(uint8_t adr);
	virtual HAL_StatusTypeDef getData(float *temperature, float *humidity);
	virtual void showState(OutStream *strm);
	virtual void execFun(OutStream *strm, int funNr);
	virtual void showMeas(OutStream *strm);
	virtual bool isError();
};

SHT35DevPub* SHT35DevPub::createDev(uint8_t devAdr) {
	return new SHT35Dev(devAdr);
}

SHT35Dev::SHT35Dev(uint8_t adr) {
	mDevAdr = adr;
	init();
}
void SHT35Dev::init() {
	mDevExist = (I2c1Bus::checkDevMtx(mDevAdr) == HAL_OK);

	//SoftReset();
	ReadSerialNumber(&serialNr);
	mMeasStart = StartPeriodicMeasurment(REPEATAB_HIGH, FREQUENCY_2HZ);
	DisableHeater();
	filterTemp.init(FILTR_FACTOR);
	filterHumidity.init(FILTR_FACTOR);
	mLastRdDataTick = 0;
	mLastTryRdTick = 0;

}

uint16_t SHT35Dev::rdSwap(uint8_t *p) {
	uint16_t w;
	w = p[0] << 8;
	w |= p[1];
	return w;
}

uint8_t SHT35Dev::CalcCrc(uint8_t *data, uint8_t cnt) {
	uint8_t crc = 0xFF;

	for (int j = 0; j < cnt; j++) {
		crc ^= data[j];
		for (int ii = 8; ii > 0; --ii) {
			if (crc & 0x80)
				crc = (crc << 1) ^ POLYNOMIAL;
			else
				crc = (crc << 1);
		}
	}
	return crc;
}

float SHT35Dev::CalcTemperature(uint16_t rawValue) {
// calculate temperature [°C]
// T = -45 + 175 * rawValue / (2^16-1)
	return 175.0f * (float) rawValue / 65535.0f - 45.0f;
}

float SHT35Dev::CalcHumidity(uint16_t rawValue) {
// calculate relative humidity [%RH]
// RH = rawValue / (2^16-1) * 100
	return 100.0f * (float) rawValue / 65535.0f;
}

uint16_t SHT35Dev::CalcRawTemperature(float temperature) {
// calculate raw temperature [ticks]
// rawT = (temperature + 45) / 175 * (2^16-1)
	return (temperature + 45.0f) / 175.0f * 65535.0f;
}

uint16_t SHT35Dev::CalcRawHumidity(float humidity) {
// calculate raw relative humidity [ticks]
// rawRH = humidity / 100 * (2^16-1)
	return humidity / 100.0f * 65535.0f;
}

HAL_StatusTypeDef SHT35Dev::ReadSerialNumber(uint32_t *serialNumber) {
	HAL_StatusTypeDef st;
	if (I2c1Bus::openMutex(10, 100)) {
		uint8_t data[6];

		st = HAL_I2C_Mem_Read(&I2c1Bus::hi2c, mDevAdr, CMD_READ_SERIALNBR, I2C_MEMADD_SIZE_16BIT, data, 6, 100);
		if (st == HAL_OK) {
			uint8_t crc1 = CalcCrc(&data[0], 2);
			uint8_t crc2 = CalcCrc(&data[3], 2);
			if (crc1 == data[2] && crc2 == data[5]) {
				uint32_t sn;
				sn = data[0] << 24;
				sn |= data[1] << 16;
				sn |= data[3] << 8;
				sn |= data[4];
				*serialNumber = sn;
			} else {
				*serialNumber = 0;
				st = HAL_CRC_ERR;
			}
		}

		I2c1Bus::closeMutex();
	} else
		st = HAL_NO_SEMF;
	return st;
}

HAL_StatusTypeDef SHT35Dev::ReadWordWithCrc(uint16_t cmdSHT, uint16_t *val) {
	HAL_StatusTypeDef st;
	if (I2c1Bus::openMutex(11, 100)) {
		uint8_t data[3];

		st = HAL_I2C_Mem_Read(&I2c1Bus::hi2c, mDevAdr, cmdSHT, I2C_MEMADD_SIZE_16BIT, data, 3, 100);
		if (st == HAL_OK) {
			uint8_t crc1 = CalcCrc(&data[0], 2);
			if (crc1 == data[2]) {
				*val = rdSwap(data);
			} else {
				*val = 0;
				st = HAL_CRC_ERR;
			}
		}
		I2c1Bus::closeMutex();
	} else
		st = HAL_NO_SEMF;
	return st;
}

HAL_StatusTypeDef SHT35Dev::ReadStatus(uint16_t *status) {
	return ReadWordWithCrc(CMD_READ_STATUS, status);
}

HAL_StatusTypeDef SHT35Dev::writeCmd(uint16_t cmd) {
	HAL_StatusTypeDef st;
	if (I2c1Bus::openMutex(12, 100)) {
		uint8_t data[2];
		data[0] = cmd >> 8;
		data[1] = cmd;

		st = HAL_I2C_Master_Transmit(&I2c1Bus::hi2c, mDevAdr, data, 2, 100);
		I2c1Bus::closeMutex();
	} else
		st = HAL_NO_SEMF;
	return st;
}

HAL_StatusTypeDef SHT35Dev::EnableHeater(void) {
	HAL_StatusTypeDef st = writeCmd(CMD_HEATER_ENABLE);
	osDelay(5);
	return st;
}

HAL_StatusTypeDef SHT35Dev::DisableHeater(void) {
	HAL_StatusTypeDef st = writeCmd(CMD_HEATER_DISABLE);
	osDelay(5);
	return st;

}

HAL_StatusTypeDef SHT35Dev::ClearAllAlertFlags(void) {
	return writeCmd(CMD_CLEAR_STATUS);
}

HAL_StatusTypeDef SHT35Dev::SoftReset(void) {
	return writeCmd(CMD_SOFT_RESET);
}

HAL_StatusTypeDef SHT35Dev::SoftBreak(void) {
	return writeCmd(CMD_BREAK);
}

HAL_StatusTypeDef SHT35Dev::SendAlertData(uint16_t cmdSHT, float humidity, float temperature) {
	if ((humidity < 0.0f) || (humidity > 100.0f) || (temperature < -45.0f) || (temperature > 130.0f)) {
		return HAL_DATA_ERR;
	} else {
		uint16_t rawHumidity = CalcRawHumidity(humidity);
		uint16_t rawTemperature = CalcRawTemperature(temperature);
		uint16_t w = (rawHumidity & 0xFE00) | ((rawTemperature >> 7) & 0x001FF);
		uint8_t tab[3];
		tab[0] = w >> 8;
		tab[1] = w & 0xff;
		tab[2] = CalcCrc(tab, 2);
		return HAL_I2C_Mem_Write(&I2c1Bus::hi2c, mDevAdr, cmdSHT, I2C_MEMADD_SIZE_16BIT, tab, 3, 100);
	}
}

HAL_StatusTypeDef SHT35Dev::ReadAlertData(uint16_t cmdSHT, float *humidity, float *temperature) {
	uint16_t data;
	HAL_StatusTypeDef st = ReadWordWithCrc(cmdSHT, &data);
	if (st == HAL_OK) {

		*humidity = CalcHumidity(data & 0xFE00);
		*temperature = CalcTemperature(data << 7);
	}
	return st;

}

HAL_StatusTypeDef SHT35Dev::setAlerts(AlertRec *rec) {
	HAL_StatusTypeDef st = HAL_NO_SEMF;
	if (I2c1Bus::openMutex(13, 100)) {

		st = SendAlertData(CMD_W_AL_LIM_HS, rec->humidityHighSet, rec->temperatureHighSet);
		if (st == HAL_OK)
			st = SendAlertData(CMD_W_AL_LIM_HC, rec->humidityHighClear, rec->temperatureHighClear);
		if (st == HAL_OK)
			st = SendAlertData(CMD_W_AL_LIM_LC, rec->humidityLowClear, rec->temperatureLowClear);
		if (st == HAL_OK)
			st = SendAlertData(CMD_W_AL_LIM_LS, rec->humidityLowSet, rec->temperatureLowSet);
		I2c1Bus::closeMutex();
	}
	return st;

}

HAL_StatusTypeDef SHT35Dev::readAlerts(AlertRec *rec) {
	HAL_StatusTypeDef st;
	st = ReadAlertData(CMD_W_AL_LIM_HS, &rec->humidityHighSet, &rec->temperatureHighSet);
	if (st == HAL_OK)
		st = ReadAlertData(CMD_W_AL_LIM_HC, &rec->humidityHighClear, &rec->temperatureHighClear);
	if (st == HAL_OK)
		st = ReadAlertData(CMD_W_AL_LIM_LC, &rec->humidityLowClear, &rec->temperatureLowClear);
	if (st == HAL_OK)
		st = ReadAlertData(CMD_W_AL_LIM_LS, &rec->humidityLowSet, &rec->temperatureLowSet);
	return st;

}

const uint16_t TabStartCmd[3][5] = { //
		{ CMD_MEAS_PERI_05_H, CMD_MEAS_PERI_1_H, CMD_MEAS_PERI_2_H, CMD_MEAS_PERI_4_H, CMD_MEAS_PERI_10_H }, //
				{ CMD_MEAS_PERI_05_M, CMD_MEAS_PERI_1_M, CMD_MEAS_PERI_2_M, CMD_MEAS_PERI_4_M, CMD_MEAS_PERI_10_M }, //
				{ CMD_MEAS_PERI_05_L, CMD_MEAS_PERI_1_L, CMD_MEAS_PERI_2_L, CMD_MEAS_PERI_4_L, CMD_MEAS_PERI_10_L } //
		};

HAL_StatusTypeDef SHT35Dev::StartPeriodicMeasurment(etRepeatability repeatability, etFrequency frequency) {
	HAL_StatusTypeDef st;

	if (repeatability >= 0 && repeatability < 3 && frequency >= 0 && frequency < 5) {
		uint16_t cmdSHT = TabStartCmd[repeatability][frequency];
		st = writeCmd(cmdSHT);
	} else
		st = HAL_DATA_ERR;
	return st;
}

bool SHT35Dev::isError() {
	return (HAL_GetTick() - mLastRdDataTick > TIME_DT_VALID);
}

HAL_StatusTypeDef SHT35Dev::getData(float *temperature, float *humidity) {
	*temperature = filterTemp.get();
	*humidity = filterHumidity.get();
	return HAL_OK;
}

HAL_StatusTypeDef SHT35Dev::readData() {

	meas.temp = NAN;
	meas.humi = NAN;
	HAL_StatusTypeDef st = HAL_BUSY;
	if (I2c1Bus::openMutex(14, 100)) {

		uint8_t data[6];
		st = HAL_I2C_Mem_Read(&I2c1Bus::hi2c, mDevAdr, CMD_FETCH_DATA, I2C_MEMADD_SIZE_16BIT, data, 6, 100);

		if (st == HAL_OK) {
			uint8_t crc1 = CalcCrc(&data[0], 2);
			uint8_t crc2 = CalcCrc(&data[3], 2);
			if (crc1 == data[2] && crc2 == data[5]) {
				uint16_t rawTemp = rdSwap(&data[0]);
				uint16_t rawHumi = rdSwap(&data[3]);
				meas.temp = CalcTemperature(rawTemp);
				meas.humi = CalcHumidity(rawHumi);
				meas.mReadTick = HAL_GetTick();
			} else {
				st = HAL_CRC_ERR;
			}
		}
		I2c1Bus::closeMutex();
	}
	filterTemp.inp(meas.temp);
	filterHumidity.inp(meas.humi);
	return st;
}

void SHT35Dev::tick() {
	uint32_t tt = HAL_GetTick();
	if (tt - mLastRdDataTick > TIME_DT_RD) {
		if (tt - mLastTryRdTick > 200) {
			mLastTryRdTick = tt;
			if (readData() == HAL_OK) {
				mLastRdDataTick = tt;
			}
		}
	}
}

void SHT35Dev::execFun(OutStream *strm, int funNr) {
	AlertRec alert;

	HAL_StatusTypeDef st;
	switch (funNr) {
	case 0:
		st = readAlerts(&alert);
		if (st == HAL_OK) {
			strm->oMsg("TempHigh: %.0f-%.0f", alert.temperatureHighClear, alert.temperatureHighSet);
			strm->oMsg("TempLow : %.0f-%.0f", alert.temperatureLowClear, alert.temperatureLowSet);
			strm->oMsg("HumiHigh: %.0f-%.0f", alert.humidityHighClear, alert.humidityHighSet);
			strm->oMsg("HumiLow : %.0f-%.0f", alert.humidityLowClear, alert.humidityLowSet);
		} else
			strm->oMsg("readAlerts error:%s", HAL_getErrStr(st));
		break;
	case 1:
		alert.temperatureHighSet = 30;
		alert.temperatureHighClear = 25;
		alert.temperatureLowClear = 10;
		alert.temperatureLowSet = 5;
		alert.humidityHighSet = 80;
		alert.humidityHighClear = 75;
		alert.humidityLowClear = 60;
		alert.humidityLowSet = 55;

		strm->oMsg("setAlerts st=%s", HAL_getErrStr(setAlerts(&alert)));
		break;
	case 2:
		alert.temperatureHighSet = 33;
		alert.temperatureHighClear = 28;
		alert.temperatureLowClear = 13;
		alert.temperatureLowSet = 8;
		alert.humidityHighSet = 82;
		alert.humidityHighClear = 77;
		alert.humidityLowClear = 62;
		alert.humidityLowSet = 57;

		strm->oMsg("setAlerts st=%s", HAL_getErrStr(setAlerts(&alert)));
		break;
	case 3:
		strm->oMsg("EnableHeater st=%s", HAL_getErrStr(EnableHeater()));
		break;
	case 4:
		strm->oMsg("DisableHeater st=%s", HAL_getErrStr(DisableHeater()));
		break;
	case 5:
		showStatus(strm);
		break;
	case 6:
		strm->oMsg("ClearAllAlertFlags st=%s", HAL_getErrStr(ClearAllAlertFlags()));
		break;
	case 7:
		strm->oMsg("SoftReset st=%s", HAL_getErrStr(SoftReset()));
		break;
	case 8:
		st = StartPeriodicMeasurment(REPEATAB_HIGH, FREQUENCY_4HZ);
		strm->oMsg("StartPeriodicMeasurment st=%s", HAL_getErrStr(st));
		break;
	case 9:
		showMeasData(strm);
		break;
	case 10:
		showDevExist(strm);
		break;
	case 11:
		showSerialNumer(strm);
		break;

	}
}

void SHT35Dev::showMeasData(OutStream *strm) {
	float temperature, humidity;
	HAL_StatusTypeDef st = getData(&temperature, &humidity);
	if (st == HAL_OK)
		strm->oMsg("Temper=%.2f[st]  Humi=%.1f[%%]", temperature, humidity);
	else
		strm->oMsg("Data error:%s", HAL_getErrStr(st));
}

void SHT35Dev::showStatus(OutStream *strm) {
	uint16_t status;
	HAL_StatusTypeDef st = ReadStatus(&status);
	if (st == HAL_OK) {
		status &= 0xAC13; // maskowanie pól reserved
		strm->oMsg("Status=0x%04X", status);
	} else
		strm->oMsg("status error:%s", HAL_getErrStr(st));
}

void SHT35Dev::showSerialNumer(OutStream *strm) {
	uint32_t sn;
	HAL_StatusTypeDef st = ReadSerialNumber(&sn);
	if (st == HAL_OK)
		strm->oMsg("SerialNb=0x%08X", sn);
	else
		strm->oMsg("SerialNb error:%s", HAL_getErrStr(st));
}

void SHT35Dev::showMeas(OutStream *strm) {
	float temperature, humidity;
	HAL_StatusTypeDef st = getData(&temperature, &humidity);
	if (st == HAL_OK)
		strm->oMsg("SHT35 : Temper=%.2f[st]  Humi=%.1f[%%]", temperature, humidity);
	else
		strm->oMsg("SHT35 : Data error:%s", HAL_getErrStr(st));
}

void SHT35Dev::showState(OutStream *strm) {
	strm->oMsg("__SHT35Dev__");
	strm->oMsg("chipExist: %s", YN(mDevExist));
	if (mDevExist) {
		strm->oMsg("SerialNb=0x%08X", serialNr);
		strm->oMsg("MeasStart=%s", HAL_getErrStr(mMeasStart));
		showDevExist(strm);
		showStatus(strm);
		showMeasData(strm);
	}
}

//-------------------------------------------------------------------------------------------------------------------------
// Bmp338Dev
//-------------------------------------------------------------------------------------------------------------------------
typedef struct {
	uint16_t T1, T2, P5, P6;
	int16_t P1, P2, P9;
	int8_t T3, P3, P4, P7, P8, P10, P11;
	double T1f, T2f, T3f, P1f, P2f, P3f, P4f, P5f, P6f, P7f, P8f, P9f, P10f, P11f;
} Calibration;

#define AC_CONCAT_BYTES(msb, lsb) (((uint16_t)msb << 8) | (uint16_t)lsb)

class Bmp338Dev: public Bmp338DevPub {
private:
	enum {
		REG_CHIPID = 0x00, //
		REG_CAL_DATA = 0x31, //
		REG_CMD = 0x7E, //
		REG_CONTROL = 0x1B, //
		REG_STATUS = 0x03, //
		REG_PRESS = 0x04, //
	};

	enum {
		CHIPID = 0x50, // wartość w rejestrze REG_CHIP_ID
		CMD_RESET = 0xB6, //
		CMD_MEASURE = 0x13, //  rozkaz wykonania pomiaru
		STATUS = 0x60, //
	};

	Calibration _c;
	HAL_StatusTypeDef read_coefficients();
	uint32_t mLastRdDataTick;
	uint32_t mLastTryRdTick;

	double ac_pow(double base, uint8_t power);

	double comp_temperature(uint32_t T);
	double comp_pressure(uint32_t P, double T);
	HAL_StatusTypeDef reset();
	HAL_StatusTypeDef measure(double *temp, double *pressure);
	void showMeasData(OutStream *strm);
	HAL_StatusTypeDef Start();
	DtFilter filterTemper;
	DtFilter filterPressure;
	HAL_StatusTypeDef readData();
protected:
	virtual void tick();
	virtual void init();
public:
	Bmp338Dev(uint8_t adr);
	virtual HAL_StatusTypeDef getData(float *temperature, float *pressure);
	virtual void showState(OutStream *strm);
	virtual void execFun(OutStream *strm, int funNr);
	virtual void showMeas(OutStream *strm);
	virtual bool isError();

};

Bmp338DevPub* Bmp338DevPub::createDev(uint8_t devAdr) {
	return new Bmp338Dev(devAdr);
}

Bmp338Dev::Bmp338Dev(uint8_t adr) {
	mDevAdr = adr;
	init();
}

void Bmp338Dev::init() {
	chipIdOk = false;
	coefOk = false;
	Start();
	filterTemper.init(FILTR_FACTOR);
	filterPressure.init(FILTR_FACTOR);
	mLastRdDataTick = HAL_GetTick();
	mLastTryRdTick = HAL_GetTick();
}

HAL_StatusTypeDef Bmp338Dev::Start() {
	HAL_StatusTypeDef st = HAL_NO_SEMF;
	mDevExist = (I2c1Bus::checkDevMtx(mDevAdr) == HAL_OK);
	if (mDevExist) {
		if (I2c1Bus::openMutex(20, 100)) {
			uint8_t b;
			st = I2c1Bus::readByte(mDevAdr, REG_CHIPID, &b);
			if (st == HAL_OK) {
				chipIdOk = (b == CHIPID);
				reset();
				coefOk = (read_coefficients() == HAL_OK);
			}
		}
		I2c1Bus::closeMutex();
	}
	return st;
}

HAL_StatusTypeDef Bmp338Dev::reset() {
	return I2c1Bus::writeByte(mDevAdr, REG_CMD, CMD_RESET);
}

HAL_StatusTypeDef Bmp338Dev::read_coefficients() {
	const int size = 21;
	uint8_t b[size];

	HAL_StatusTypeDef st = HAL_I2C_Mem_Read(&I2c1Bus::hi2c, mDevAdr, REG_CAL_DATA, I2C_MEMADD_SIZE_8BIT, b, size, 100);

	_c.T1 = (uint16_t) AC_CONCAT_BYTES(b[1], b[0]);
	_c.T2 = (uint16_t) AC_CONCAT_BYTES(b[3], b[2]);
	_c.T3 = (int8_t) b[4];
	_c.P1 = (int16_t) AC_CONCAT_BYTES(b[6], b[5]);
	_c.P2 = (int16_t) AC_CONCAT_BYTES(b[8], b[7]);
	_c.P3 = (int8_t) b[9];
	_c.P4 = (int8_t) b[10];
	_c.P5 = (uint16_t) AC_CONCAT_BYTES(b[12], b[11]);
	_c.P6 = (uint16_t) AC_CONCAT_BYTES(b[14], b[13]);
	_c.P7 = (int8_t) b[15];
	_c.P8 = (int8_t) b[16];
	_c.P9 = (int16_t) AC_CONCAT_BYTES(b[18], b[17]);
	_c.P10 = (int8_t) b[19];
	_c.P11 = (int8_t) b[20];

	_c.T1f = (double) _c.T1 / 0.00390625f;
	_c.T2f = (double) _c.T2 / 1073741824.0f;
	_c.T3f = (double) _c.T3 / 281474976710656.0f;
	_c.P1f = ((double) _c.P1 - 16384) / 1048576.0f;
	_c.P2f = ((double) _c.P2 - 16384) / 536870912.0f;
	_c.P3f = (double) _c.P3 / 4294967296.0f;
	_c.P4f = (double) _c.P4 / 137438953472.0f;
	_c.P5f = (double) _c.P5 / 0.125f;
	_c.P6f = (double) _c.P6 / 64.0f;
	_c.P7f = (double) _c.P7 / 256.0f;
	_c.P8f = (double) _c.P8 / 32768.0f;
	_c.P9f = (double) _c.P9 / 281474976710656.0f;
	_c.P10f = (double) _c.P10 / 281474976710656.0f;
	_c.P11f = (double) _c.P11 / 36893488147419103232.0f;

	return st;
}

double Bmp338Dev::comp_temperature(uint32_t T) {
	const double TP1 = (double) (T - _c.T1f);
	const double TP2 = (double) (TP1 * _c.T2f);
	return TP2 + (TP1 * TP1) * _c.T3f;
}

double Bmp338Dev::ac_pow(double base, uint8_t power) {
	double pow_output = 1;
	while (power != 0) {
		pow_output = base * pow_output;
		power--;
	}
	return pow_output;
}

/* Calibrate pressure reading */
double Bmp338Dev::comp_pressure(uint32_t P, double T) {
	double partial_data1;
	double partial_data2;
	double partial_data3;
	double partial_data4;
	double partial_out1;
	double partial_out2;

	partial_data1 = _c.P6f * T;
	partial_data2 = _c.P7f * ac_pow(T, 2);
	partial_data3 = _c.P8f * ac_pow(T, 3);
	partial_out1 = _c.P5f + partial_data1 + partial_data2 + partial_data3;

	partial_data1 = _c.P2f * T;
	partial_data2 = _c.P3f * ac_pow(T, 2);
	partial_data3 = _c.P4f * ac_pow(T, 3);
	partial_out2 = P * (_c.P1f + partial_data1 + partial_data2 + partial_data3);

	partial_data1 = ac_pow((double) P, 2);
	partial_data2 = _c.P9f + _c.P10f * T;
	partial_data3 = partial_data1 * partial_data2;
	partial_data4 = partial_data3 + ac_pow((double) P, 3) * _c.P11f;

	return partial_out1 + partial_out2 + partial_data4;
}

HAL_StatusTypeDef Bmp338Dev::measure(double *temp, double *press) {
	HAL_StatusTypeDef st = HAL_NO_SEMF;

	if (I2c1Bus::openMutex(21, 100)) {
		st = I2c1Bus::writeByte(mDevAdr, REG_CONTROL, CMD_MEASURE);
		if (st == HAL_OK) {

			int loop_count = 0;
			while (1) {
				loop_count++;
				osDelay(200);
				uint8_t v = 0;
				st = I2c1Bus::readByte(mDevAdr, REG_STATUS, &v);
				if (st != HAL_OK)
					break;
				if (v == 0x70) {
					break;
				}
				if (loop_count > 10) {
					st = HAL_TIMEOUT;
					break;
				}
			}

			if (st == HAL_OK) {
				uint8_t p[6];
				st = I2c1Bus::readBytes(mDevAdr, REG_PRESS, 6, p);
				if (st == HAL_OK) {

					uint32_t P = (p[2] << 16) | (p[1] << 8) | p[0];
					uint32_t T = (p[5] << 16) | (p[4] << 8) | p[3];

					double temperature = comp_temperature(T);
					double pressure = comp_pressure(P, temperature);

					*temp = temperature;
					*press = pressure / 100;  //hPa
				}
			}
		}
		I2c1Bus::closeMutex();
	}
	if (st != HAL_OK) {
		*temp = NAN;
		*press = NAN;
	}
	return st;
}

bool Bmp338Dev::isError() {
	return (HAL_GetTick() - mLastRdDataTick > TIME_DT_VALID);
}

HAL_StatusTypeDef Bmp338Dev::getData(float *temperature, float *pressure) {
	if (!isError()) {
		*temperature = filterTemper.get();
		*pressure = filterPressure.get();
		return HAL_OK;
	} else {
		*temperature = NAN;
		*pressure = NAN;
		return HAL_ERROR;
	}
}

HAL_StatusTypeDef Bmp338Dev::readData() {
	double d_temperature, d_pressure;
	HAL_StatusTypeDef st = measure(&d_temperature, &d_pressure);
	if (st == HAL_OK) {
		filterTemper.inp(d_temperature);
		filterPressure.inp(d_pressure);
	}
	return st;
}

void Bmp338Dev::tick() {
	uint32_t tt = HAL_GetTick();
	if (tt - mLastRdDataTick > TIME_DT_RD) {
		if (tt - mLastTryRdTick > 200) {
			mLastTryRdTick = tt;
			if (readData() == HAL_OK) {
				mLastRdDataTick = tt;
			}
		}
	}
}

void Bmp338Dev::execFun(OutStream *strm, int funNr) {
	switch (funNr) {
	case 20:
		strm->oMsg("BMP338 Start st=%s", HAL_getErrStr(Start()));
		strm->oMsg("chipIdOk: %s", OkErr(chipIdOk));
		strm->oMsg("coefOk: %s", OkErr(coefOk));
		break;
	}
}

void Bmp338Dev::showMeasData(OutStream *strm) {
	double temperature, pressure;
	HAL_StatusTypeDef st = measure(&temperature, &pressure);
	if (st == HAL_OK)
		strm->oMsg("Temper=%.2f[st]  Pressure=%.2f[hPa]", temperature, pressure);
	else
		strm->oMsg("Data error:%s", HAL_getErrStr(st));
}

void Bmp338Dev::showMeas(OutStream *strm) {
	double temperature, pressure;
	HAL_StatusTypeDef st = measure(&temperature, &pressure);
	if (st == HAL_OK)
		strm->oMsg("BMP338: Temper=%.2f[st]  Pressure=%.2f[hPa]", temperature, pressure);
	else
		strm->oMsg("BMP338: Data error:%s", HAL_getErrStr(st));

}

void Bmp338Dev::showState(OutStream *strm) {
	strm->oMsg("__BMP338__");
	strm->oMsg("chipExist: %s", YN(mDevExist));
	if (mDevExist) {
		strm->oMsg("chipIdOk: %s", OkErr(chipIdOk));
		strm->oMsg("coefOk: %s", OkErr(coefOk));
		showMeasData(strm);
	}
}

