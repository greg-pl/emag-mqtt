/*
 * Bmp338Device.h
 *
 *  Created on: 10 gru 2022
 *      Author: Grzegorz
 */

#ifndef BMP338DEVICE_H_
#define BMP338DEVICE_H_

#include "ProjectConfig.h"
#if(DEV_BMP338)

#include <I2cDev.h>
#include <Utils.h>



typedef struct {
	uint16_t T1, T2, P5, P6;
	int16_t P1, P2, P9;
	int8_t T3, P3, P4, P7, P8, P10, P11;
	double T1f, T2f, T3f, P1f, P2f, P3f, P4f, P5f, P6f, P7f, P8f, P9f, P10f, P11f;
} Calibration;

class Bmp338Device: public I2cDev {
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
	void showInit(OutStream *strm);

protected:
	virtual void tick();
	virtual void init();
	virtual void shell(OutStream *strm, const char *cmd);
public:
    bool chipIdOk;
    bool coefOk;
	Bmp338Device(I2cBus *bus,  uint8_t adr, const char *name);
	static void funShowState(OutStream *strm, const char *cmd, void *arg) ;
	static void funBMPInit(OutStream *strm, const char *cmd, void *arg) ;

	virtual void showState(OutStream *strm);
	virtual void showMeas(OutStream *strm);
public:
	//Unidev
	virtual bool getMeasValue(MeasType measType, float *val);
	virtual bool isMeasServiced(MeasType measType);
	virtual bool isDataError();
	virtual bool isAnyConfiguredData();

	//virtual void getDeviceStatusTxt(char *txt, int max);

};

#endif //DEV_BMP338

#endif /* BMP338DEVICE_H_ */
