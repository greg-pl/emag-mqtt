/*
 * GasS873.h
 *
 *  Created on: 7 gru 2022
 *      Author: Grzegorz
 */

#ifndef GASS873_H_
#define GASS873_H_

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



class GasS873: public MdbMasterTask {
private:

	enum {
		TM_AUTO_RD = 30000, // czas automatycznego odczytu komory gazu
		MAX_TIME_REPL = 1000, // maksymalny czas odpowiedzi
		TIME_MEAS_VALID = 60000, //
		MAX_DEV_CNT = 16,
	};

	struct {
		int phase;
		uint32_t tick;
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
		S873Data sensorTab[MAX_DEV_CNT];
	} gasData;
	struct {
		volatile bool flag;
		int cnt;
		uint16_t tab[MAX_DEV_CNT];
		int phase;
		int sensNr;
	} zeroOfs;

	float getMeasFactor(MeasType mType);
	bool execMyMenuItem(OutStream *strm, int idx, const char *cmd);
	char gtxt[200]; //używane przez "showMeas"
	void showMeas(OutStream *strm);
	MeasType  getMeasType(uint8_t sensorType, uint8_t verTyp);
protected:
	virtual void loopFunc();
	virtual void doOnTimeOut();

	virtual void onReciveData(bool replOK, uint8_t mdbFun, const uint8_t *tab, int regCnt);
	virtual void showState(OutStream *strm);
	virtual bool execMenuItem(OutStream *strm, int idx, const char *cmd);
	virtual const ShellItem* getMenu();
	virtual const char* getMenuName();

	const char* getSensValidStr(uint16_t status);
	bool isMeasValid(uint16_t status);

public:
	GasS873(int mdbNr, int portNr);
public:
	virtual bool getMeasValue(MeasType measType, float *val);
	virtual bool getMeasValue(MeasType measType, int filtrType, float *val);
	virtual bool isAnyConfiguredData();
	virtual bool isDataError();
	virtual void getDeviceStatusTxt(char *txt, int max);

	bool isError();
};

#endif /* GASS873_H_ */
