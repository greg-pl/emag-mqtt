/*
 * Bmp338Device.cpp
 *
 *  Created on: 10 gru 2022
 *      Author: Grzegorz
 */

#include <Bmp338Device.h>
#include <math.h>
#include <Config.h>

//-------------------------------------------------------------------------------------------------------------------------
// Bmp338Device
//-------------------------------------------------------------------------------------------------------------------------

extern Config *config;

#define AC_CONCAT_BYTES(msb, lsb) (((uint16_t)msb << 8) | (uint16_t)lsb)

Bmp338Device::Bmp338Device(I2cBus *bus, uint8_t adr, const char *name) :
		I2cDev::I2cDev(bus, adr, name) {
	init();
}

void Bmp338Device::init() {
	chipIdOk = false;
	coefOk = false;
	Start();
	filterTemper.init(FILTR_FACTOR);
	filterPressure.init(FILTR_FACTOR);
	mLastRdDataTick = HAL_GetTick();
	mLastTryRdTick = HAL_GetTick();
}

HAL_StatusTypeDef Bmp338Device::Start() {
	HAL_StatusTypeDef st = HAL_NO_SEMF;
	mDevExist = (checkDevMtx() == HAL_OK);
	if (mDevExist) {
		if (openMutex(20, 100)) {
			uint8_t b;
			st = readByte(REG_CHIPID, &b);
			if (st == HAL_OK) {
				chipIdOk = (b == CHIPID);
				reset();
				coefOk = (read_coefficients() == HAL_OK);
			}
		}
		closeMutex();
	}
	return st;
}

HAL_StatusTypeDef Bmp338Device::reset() {
	return writeByte(REG_CMD, CMD_RESET);
}

HAL_StatusTypeDef Bmp338Device::read_coefficients() {
	const int size = 21;
	uint8_t b[size];

	HAL_StatusTypeDef st = readBytes(REG_CAL_DATA, size, b);

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

double Bmp338Device::comp_temperature(uint32_t T) {
	const double TP1 = (double) (T - _c.T1f);
	const double TP2 = (double) (TP1 * _c.T2f);
	return TP2 + (TP1 * TP1) * _c.T3f;
}

double Bmp338Device::ac_pow(double base, uint8_t power) {
	double pow_output = 1;
	while (power != 0) {
		pow_output = base * pow_output;
		power--;
	}
	return pow_output;
}

/* Calibrate pressure reading */
double Bmp338Device::comp_pressure(uint32_t P, double T) {
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

HAL_StatusTypeDef Bmp338Device::measure(double *temp, double *press) {
	HAL_StatusTypeDef st = HAL_NO_SEMF;

	if (openMutex(21, 100)) {
		st = writeByte(REG_CONTROL, CMD_MEASURE);
		if (st == HAL_OK) {

			int loop_count = 0;
			while (1) {
				loop_count++;
				osDelay(200);
				uint8_t v = 0;
				st = readByte(REG_STATUS, &v);
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
				st = readBytes(REG_PRESS, 6, p);
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
		closeMutex();
	}
	if (st != HAL_OK) {
		*temp = NAN;
		*press = NAN;
	}
	return st;
}

bool Bmp338Device::isAnyConfiguredData() {
	return config->data.R.sensExist[ssTEMPERATURE] || config->data.R.sensExist[ssPRESSURE];
}

bool Bmp338Device::isDataError() {
	return (HAL_GetTick() - mLastRdDataTick > TIME_DT_VALID);
}


bool Bmp338Device::getMeasValue(MeasType measType, float *val) {
	if (isDataError())
		return false;
	switch (measType) {
	case ssTEMPERATURE:
		*val = filterTemper.get();
		return true;
	case ssPRESSURE:
		*val = filterPressure.get();
		return true;
	default:
		return false;
	}
}

HAL_StatusTypeDef Bmp338Device::readData() {
	double d_temperature, d_pressure;
	HAL_StatusTypeDef st = measure(&d_temperature, &d_pressure);
	if (st == HAL_OK) {
		filterTemper.inp(d_temperature);
		filterPressure.inp(d_pressure);
	}
	return st;
}

void Bmp338Device::tick() {
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

void Bmp338Device::showInit(OutStream *strm) {
	strm->oMsg("BMP338 Start st=%s", HAL_getErrStr(Start()));
	strm->oMsg("chipIdOk: %s", OkErr(chipIdOk));
	strm->oMsg("coefOk: %s", OkErr(coefOk));
}

void Bmp338Device::showMeasData(OutStream *strm) {
	double temperature, pressure;
	HAL_StatusTypeDef st = measure(&temperature, &pressure);
	if (st == HAL_OK)
		strm->oMsg("Temper=%.2f[st]  Pressure=%.2f[hPa]", temperature, pressure);
	else
		strm->oMsg("Data error:%s", HAL_getErrStr(st));
}

void Bmp338Device::showMeas(OutStream *strm) {
	double temperature, pressure;
	HAL_StatusTypeDef st = measure(&temperature, &pressure);
	if (st == HAL_OK)
		strm->oMsg("BMP338: Temper=%.2f[st]  Pressure=%.2f[hPa]", temperature, pressure);
	else
		strm->oMsg("BMP338: Data error:%s", HAL_getErrStr(st));

}

void Bmp338Device::showState(OutStream *strm) {
	strm->oMsg("__BMP338__");
	strm->oMsg("chipExist: %s", YN(mDevExist));
	if (mDevExist) {
		strm->oMsg("chipIdOk: %s", OkErr(chipIdOk));
		strm->oMsg("coefOk: %s", OkErr(coefOk));
		showMeasData(strm);
	}
}

void Bmp338Device::funShowState(OutStream *strm, const char *cmd, void *arg) {
	Bmp338Device *dev = (Bmp338Device*) arg;
	dev->showState(strm);
}

void Bmp338Device::funBMPInit(OutStream *strm, const char *cmd, void *arg) {
	Bmp338Device *dev = (Bmp338Device*) arg;
	dev->showInit(strm);
}

const ShellItemFx menuBmpFx[] = { //
		{ "s", "stan", Bmp338Device::funShowState }, //
				{ "init", "inicjuj BMP338", Bmp338Device::funBMPInit }, //
				{ NULL, NULL } };

void Bmp338Device::shell(OutStream *strm, const char *cmd) {
	execMenuCmd(strm, menuBmpFx, cmd, this, "Bmp338 Menu");
}

