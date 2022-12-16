/*
 * SHT35Device.cpp
 *
 *  Created on: 10 gru 2022
 *      Author: Grzegorz
 */

#include <SHT35Device.h>
#include <math.h>
#include <Config.h>
#include "Token.h"

extern Config *config;

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

SHT35Device::SHT35Device(I2cBus *bus, uint8_t adr, const char *name) :
		I2cDev::I2cDev(bus, adr, name) {
	init();
}

void SHT35Device::init() {
	mDevExist = (checkDevMtx() == HAL_OK);

	//SoftReset();
	ReadSerialNumber(&serialNr);
	mMeasStart = StartPeriodicMeasurment(REPEATAB_HIGH, FREQUENCY_2HZ);
	DisableHeater();
	filterTemp.init(FILTR_FACTOR);
	filterHumidity.init(FILTR_FACTOR);
	meas.mReadTick= 0;
	mLastTryRdTick = 0;

}

uint16_t SHT35Device::rdSwap(uint8_t *p) {
	uint16_t w;
	w = p[0] << 8;
	w |= p[1];
	return w;
}

uint8_t SHT35Device::CalcCrc(uint8_t *data, uint8_t cnt) {
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

float SHT35Device::CalcTemperature(uint16_t rawValue) {
// calculate temperature [°C]
// T = -45 + 175 * rawValue / (2^16-1)
	return 175.0f * (float) rawValue / 65535.0f - 45.0f;
}

float SHT35Device::CalcHumidity(uint16_t rawValue) {
// calculate relative humidity [%RH]
// RH = rawValue / (2^16-1) * 100
	return 100.0f * (float) rawValue / 65535.0f;
}

uint16_t SHT35Device::CalcRawTemperature(float temperature) {
// calculate raw temperature [ticks]
// rawT = (temperature + 45) / 175 * (2^16-1)
	return (temperature + 45.0f) / 175.0f * 65535.0f;
}

uint16_t SHT35Device::CalcRawHumidity(float humidity) {
// calculate raw relative humidity [ticks]
// rawRH = humidity / 100 * (2^16-1)
	return humidity / 100.0f * 65535.0f;
}

HAL_StatusTypeDef SHT35Device::ReadSerialNumber(uint32_t *serialNumber) {
	HAL_StatusTypeDef st;
	if (openMutex(10, 100)) {
		uint8_t data[6];
		//st = HAL_I2C_Mem_Read(&I2cBus::hi2c, mDevAdr, CMD_READ_SERIALNBR, I2C_MEMADD_SIZE_16BIT, data, 6, 100);
		st = readBytes16bit(CMD_READ_SERIALNBR, 6, data);
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

		closeMutex();
	} else
		st = HAL_NO_SEMF;
	return st;
}

HAL_StatusTypeDef SHT35Device::ReadWordWithCrc(uint16_t cmdSHT, uint16_t *val) {
	HAL_StatusTypeDef st;
	if (openMutex(11, 100)) {
		uint8_t data[3];

		//st = HAL_I2C_Mem_Read(&I2cBus::hi2c, mDevAdr, cmdSHT, I2C_MEMADD_SIZE_16BIT, data, 3, 100);
		st = readBytes16bit(cmdSHT, 3, data);

		if (st == HAL_OK) {
			uint8_t crc1 = CalcCrc(&data[0], 2);
			if (crc1 == data[2]) {
				*val = rdSwap(data);
			} else {
				*val = 0;
				st = HAL_CRC_ERR;
			}
		}
		closeMutex();
	} else
		st = HAL_NO_SEMF;
	return st;
}

HAL_StatusTypeDef SHT35Device::ReadStatus(uint16_t *status) {
	return ReadWordWithCrc(CMD_READ_STATUS, status);
}

HAL_StatusTypeDef SHT35Device::writeCmd(uint16_t cmd) {
	HAL_StatusTypeDef st;
	if (openMutex(12, 100)) {
		uint8_t data[2];
		data[0] = cmd >> 8;
		data[1] = cmd;
		//st = HAL_I2C_Master_Transmit(&I2cBus::hi2c, mDevAdr, data, 2, 100);
		st = Transmit(2, data);
		closeMutex();
	} else
		st = HAL_NO_SEMF;
	return st;
}

HAL_StatusTypeDef SHT35Device::EnableHeater(void) {
	HAL_StatusTypeDef st = writeCmd(CMD_HEATER_ENABLE);
	osDelay(5);
	return st;
}

HAL_StatusTypeDef SHT35Device::DisableHeater(void) {
	HAL_StatusTypeDef st = writeCmd(CMD_HEATER_DISABLE);
	osDelay(5);
	return st;

}

HAL_StatusTypeDef SHT35Device::ClearAllAlertFlags(void) {
	return writeCmd(CMD_CLEAR_STATUS);
}

HAL_StatusTypeDef SHT35Device::SoftReset(void) {
	return writeCmd(CMD_SOFT_RESET);
}

HAL_StatusTypeDef SHT35Device::SoftBreak(void) {
	return writeCmd(CMD_BREAK);
}

HAL_StatusTypeDef SHT35Device::SendAlertData(uint16_t cmdSHT, float humidity, float temperature) {
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
		return writeBytes16bit(cmdSHT, 3, tab);
		//return HAL_I2C_Mem_Write(&I2cBus::hi2c, mDevAdr, cmdSHT, I2C_MEMADD_SIZE_16BIT, tab, 3, 100);
	}
}

HAL_StatusTypeDef SHT35Device::ReadAlertData(uint16_t cmdSHT, float *humidity, float *temperature) {
	uint16_t data;
	HAL_StatusTypeDef st = ReadWordWithCrc(cmdSHT, &data);
	if (st == HAL_OK) {
		*humidity = CalcHumidity(data & 0xFE00);
		*temperature = CalcTemperature(data << 7);
	}
	return st;

}

HAL_StatusTypeDef SHT35Device::setAlerts(AlertRec *rec) {
	HAL_StatusTypeDef st = HAL_NO_SEMF;
	if (openMutex(13, 100)) {

		st = SendAlertData(CMD_W_AL_LIM_HS, rec->humidityHighSet, rec->temperatureHighSet);
		if (st == HAL_OK)
			st = SendAlertData(CMD_W_AL_LIM_HC, rec->humidityHighClear, rec->temperatureHighClear);
		if (st == HAL_OK)
			st = SendAlertData(CMD_W_AL_LIM_LC, rec->humidityLowClear, rec->temperatureLowClear);
		if (st == HAL_OK)
			st = SendAlertData(CMD_W_AL_LIM_LS, rec->humidityLowSet, rec->temperatureLowSet);
		closeMutex();
	}
	return st;

}

HAL_StatusTypeDef SHT35Device::readAlerts(AlertRec *rec) {
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

HAL_StatusTypeDef SHT35Device::StartPeriodicMeasurment(etRepeatability repeatability, etFrequency frequency) {
	HAL_StatusTypeDef st;

	if (repeatability >= 0 && repeatability < 3 && frequency >= 0 && frequency < 5) {
		uint16_t cmdSHT = TabStartCmd[repeatability][frequency];
		st = writeCmd(cmdSHT);
	} else
		st = HAL_DATA_ERR;
	return st;
}

bool SHT35Device::isDataError() {
	return (HAL_GetTick() - meas.mReadTick > TIME_DT_VALID);
}

bool SHT35Device::isAnyConfiguredData() {
	return config->data.R.sensExist[ssTEMPERATURE] || config->data.R.sensExist[ssHUMIDITY];
}

bool SHT35Device::getMeasValue(MeasType measType, float *val) {
	if (isDataError())
		return false;
	switch (measType) {
	case ssTEMPERATURE:
		*val = filterTemp.get();
		return true;
	case ssHUMIDITY:
		*val = filterHumidity.get();
		return true;
	default:
		return false;
	}

}

HAL_StatusTypeDef SHT35Device::readData() {

	meas.temp = NAN;
	meas.humi = NAN;
	HAL_StatusTypeDef st = HAL_BUSY;
	if (openMutex(14, 100)) {

		uint8_t data[6];
		//st = HAL_I2C_Mem_Read(&I2cBus::hi2c, mDevAdr, CMD_FETCH_DATA, I2C_MEMADD_SIZE_16BIT, data, 6, 100);
		HAL_StatusTypeDef st = readBytes16bit(CMD_FETCH_DATA, 6, data);

		if (st == HAL_OK) {
			uint8_t crc1 = CalcCrc(&data[0], 2);
			uint8_t crc2 = CalcCrc(&data[3], 2);
			if (crc1 == data[2] && crc2 == data[5]) {
				uint16_t rawTemp = rdSwap(&data[0]);
				uint16_t rawHumi = rdSwap(&data[3]);
				meas.temp = CalcTemperature(rawTemp);
				meas.humi = CalcHumidity(rawHumi);
				meas.mReadTick = HAL_GetTick();
				st = HAL_OK;
			} else {
				st = HAL_CRC_ERR;
			}
		}
		closeMutex();
	}
	filterTemp.inp(meas.temp);
	filterHumidity.inp(meas.humi);
	return st;
}

void SHT35Device::tick() {
	uint32_t tt = HAL_GetTick();
	if (tt - meas.mReadTick > TIME_DT_RD) {
		if (tt - mLastTryRdTick > 200) {
			mLastTryRdTick = tt;
			readData();
		}
	}
}

void SHT35Device::showAlert(OutStream *strm) {
	AlertRec alert;

	HAL_StatusTypeDef st = readAlerts(&alert);
	if (st == HAL_OK) {
		strm->oMsg("TempHigh: %.0f-%.0f", alert.temperatureHighClear, alert.temperatureHighSet);
		strm->oMsg("TempLow : %.0f-%.0f", alert.temperatureLowClear, alert.temperatureLowSet);
		strm->oMsg("HumiHigh: %.0f-%.0f", alert.humidityHighClear, alert.humidityHighSet);
		strm->oMsg("HumiLow : %.0f-%.0f", alert.humidityLowClear, alert.humidityLowSet);
	} else
		strm->oMsg("readAlerts error:%s", HAL_getErrStr(st));
}

void SHT35Device::setAllert(OutStream *strm, int nr) {
	AlertRec alert;
	switch (nr) {
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
	}
}

void SHT35Device::showStatus(OutStream *strm) {
	uint16_t status;
	HAL_StatusTypeDef st = ReadStatus(&status);
	if (st == HAL_OK) {
		status &= 0xAC13; // maskowanie pól reserved
		strm->oMsg("Status=0x%04X", status);
	} else
		strm->oMsg("status error:%s", HAL_getErrStr(st));
}

void SHT35Device::showSerialNumer(OutStream *strm) {
	uint32_t sn;
	HAL_StatusTypeDef st = ReadSerialNumber(&sn);
	if (st == HAL_OK)
		strm->oMsg("SerialNb=0x%08X", sn);
	else
		strm->oMsg("SerialNb error:%s", HAL_getErrStr(st));
}

void SHT35Device::showMeas(OutStream *strm) {
	if (!isDataError())
		strm->oMsg("SHT35 : Temper=%.2f[st]  Humi=%.1f[%%]", filterTemp.get(), filterHumidity.get());
	else
		strm->oMsg("SHT35 : Data error");
}

void SHT35Device::showState(OutStream *strm) {
	strm->oMsg("__SHT35Dev__");
	strm->oMsg("chipExist: %s", YN(mDevExist));
	if (mDevExist) {
		strm->oMsg("SerialNb=0x%08X", serialNr);
		strm->oMsg("MeasStart=%s", HAL_getErrStr(mMeasStart));
		showStatus(strm);
		showMeas(strm);
	}
}

void SHT35Device::funHeater(OutStream *strm, const char *cmd, void *arg) {
	SHT35Device *dev = (SHT35Device*) arg;
	bool enab = false;
	Token::getAsBool(&cmd, &enab);

	if (enab)
		strm->oMsg("EnableHeater st=%s", HAL_getErrStr(dev->EnableHeater()));
	else
		strm->oMsg("DisableHeater st=%s", HAL_getErrStr(dev->DisableHeater()));

}

void SHT35Device::funShowAlert(OutStream *strm, const char *cmd, void *arg) {
	SHT35Device *dev = (SHT35Device*) arg;
	dev->showAlert(strm);
}

void SHT35Device::funSetAllert(OutStream *strm, const char *cmd, void *arg) {
	SHT35Device *dev = (SHT35Device*) arg;
	int idx = 1;
	Token::getAsInt(&cmd, &idx);
	dev->setAllert(strm, idx);
}

void SHT35Device::funStatus(OutStream *strm, const char *cmd, void *arg) {
	SHT35Device *dev = (SHT35Device*) arg;
	dev->showStatus(strm);
}

void SHT35Device::funClear(OutStream *strm, const char *cmd, void *arg) {
	SHT35Device *dev = (SHT35Device*) arg;
	strm->oMsg("ClearAllAlertFlags st=%s", HAL_getErrStr(dev->ClearAllAlertFlags()));
}

void SHT35Device::funReset(OutStream *strm, const char *cmd, void *arg) {
	SHT35Device *dev = (SHT35Device*) arg;
	strm->oMsg("SoftReset st=%s", HAL_getErrStr(dev->SoftReset()));
}

void SHT35Device::funStart(OutStream *strm, const char *cmd, void *arg) {
	SHT35Device *dev = (SHT35Device*) arg;

	HAL_StatusTypeDef st = dev->StartPeriodicMeasurment(REPEATAB_HIGH, FREQUENCY_4HZ);
	strm->oMsg("StartPeriodicMeasurment st=%s", HAL_getErrStr(st));
}
void SHT35Device::funSerNum(OutStream *strm, const char *cmd, void *arg) {
	SHT35Device *dev = (SHT35Device*) arg;
	dev->showSerialNumer(strm);
}

const ShellItemFx menuShtFx[] = { //
		{ "status", "Pokaż rejestr statusu", SHT35Device::funStatus }, //
				{ "clr", "Czyść flagi alertów", SHT35Device::funClear }, //
				{ "heater", "wł/wył heater. use: heater 0|1", SHT35Device::funHeater }, //
				{ "show_alert", "Pokaż allerty ", SHT35Device::funShowAlert }, //
				{ "set_alert", "Ustaw zestaw alertów, parametr: 1|2. ", SHT35Device::funSetAllert }, //
				{ "reset", "Software reset", SHT35Device::funReset }, //
				{ "start", "Start pomiarów", SHT35Device::funStart }, //
				{ "sn", "Czytaj serial number", SHT35Device::funSerNum }, //
				{ NULL, NULL } };

void SHT35Device::shell(OutStream *strm, const char *cmd) {
	execMenuCmd(strm, menuShtFx, cmd, this, "SHT35 Menu");
}

