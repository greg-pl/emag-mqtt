/*
 * SHT35Device.h
 *
 *  Created on: 10 gru 2022
 *      Author: Grzegorz
 */

#ifndef SHT35DEVICE_H_
#define SHT35DEVICE_H_

#include <I2cDev.h>
#include <Utils.h>

//-------------------------------------------------------------------------------------------------------------------------
// SHT35Dev
//-------------------------------------------------------------------------------------------------------------------------

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

class SHT35Device: public I2cDev {
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
	void showAlert(OutStream *strm);
	void setAllert(OutStream *strm, int nr);


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
	uint32_t mLastTryRdTick;
	HAL_StatusTypeDef readData();
protected:
	virtual void tick();
	virtual void init();
	virtual void shell(OutStream *strm, const char *cmd);
public:
	uint32_t serialNr;
	HAL_StatusTypeDef mMeasStart;

	static void funHeater(OutStream *strm, const char *cmd, void *arg);
	static void funShowAlert(OutStream *strm, const char *cmd, void *arg);
	static void funSetAllert(OutStream *strm, const char *cmd, void *arg);
	static void funStatus(OutStream *strm, const char *cmd, void *arg);
	static void funClear(OutStream *strm, const char *cmd, void *arg);
	static void funReset(OutStream *strm, const char *cmd, void *arg);
	static void funStart(OutStream *strm, const char *cmd, void *arg);
	static void funSerNum(OutStream *strm, const char *cmd, void *arg);

public:
	SHT35Device(I2cBus *bus, uint8_t adr, const char *name);
	virtual void showState(OutStream *strm);
	virtual void showMeas(OutStream *strm);

public:
	//Unidev
	virtual bool getMeasValue(MeasType measType, float *val);
	virtual bool isAnyConfiguredData();
	virtual bool isDataError();
	//virtual void getDeviceStatusTxt(char *txt, int max);

};

#endif /* SHT35DEVICE_H_ */
