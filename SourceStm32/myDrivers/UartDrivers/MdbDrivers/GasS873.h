/*
 * GasS873.h
 *
 *  Created on: 7 gru 2022
 *      Author: Grzegorz
 */

#ifndef GASS873_H_
#define GASS873_H_

#include "Projectconfig.h"
#if (DEV_S873)


#include <MdbMasterTask.h>
#include <Config.h>


typedef struct {
	MeasType measType;
	int SensorType;
	int VerTyp;
	int Status;
	int valueHd;
	float valueFiz;
	FiltrFIR *filtrFIR;
	FiltrIR *filtrIR;
} S873Data;

class GasS873: public MdbDev {
private:

	enum {
		TM_AUTO_RD = 2000, // czas automatycznego odczytu komory gazu
		MAX_TIME_REPL = 500, // maksymalny czas odpowiedzi
		TIME_MEAS_VALID = 5000, //
		MAX_SENSOR_CNT = 16,
	};

	struct {
		int phase;
		uint32_t startTick;
		uint32_t redTick;
		int redCnt; // licznik odpowiedzi o pomiary
		int reqCnt; // licznik zapytań o pomiary
		char statusTxt[20];
	} autoRd;

	struct {
		int devCnt; // ilość odczytana
		int devCntTh; //ilośc odczytana ograniczona do MAX_DEV_CNT
		int serialNum;
		int ProdYear;
		int FirmwareVer;
		int FailureCode;
		S873Data sensorTab[MAX_SENSOR_CNT];
	} gasData;


	float getMeasFactor(MeasType mType);
	char gtxt[200]; //używane przez "showMeas"
	void showMeas(OutStream *strm);
	MeasType  getMeasType(uint8_t sensorType, uint8_t verTyp);
protected:
	virtual void loopFunc();
	virtual void onTimeOut();
	virtual void onReciveData(bool replOK, uint8_t mdbFun, const uint8_t *tab, int regCnt);

	virtual void showState(OutStream *strm);
	virtual void shell(OutStream *strm, const char *cmd);
	const char* getSensValidStr(uint16_t status);
	bool isMeasValid(uint16_t status);

public:
	GasS873(MdbMasterTask *mdbTask, uint8_t mdbAdr, const char *name);
	static void funShowMeasure(OutStream *strm, const char *cmd, void *arg);
	static void funShowState(OutStream *strm, const char *cmd, void *arg);
public:
	//Unidev
	virtual bool getMeasValue(MeasType measType, float *val);
	virtual bool isMeasServiced(MeasType measType);
	virtual bool isDataError();
	virtual void getDeviceStatusTxt(char *txt, int max);
};

#endif // DEV_S873
#endif /* GASS873_H_ */
