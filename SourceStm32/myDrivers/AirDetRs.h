/*
 * AirDetRs.h
 *
 *  Created on: 8 gru 2022
 *      Author: Grzegorz
 */

#ifndef AIRDETRS_H_
#define AIRDETRS_H_

#include <MdbMasterTask.h>

typedef struct {
	int SensorType;
	int VerTyp;
	int Status;
	int valueHd; //[ppb]
	float valueFiz; //[ug/m3]
	FiltrFIR *filtrFIR;
	FiltrIR *filtrIR;
} SensorData;

class AirDetRs: public MdbMasterTask {
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
		SensorData sensorTab[MAX_DEV_CNT];
	} gasData;
	struct {
		volatile bool flag;
		int cnt;
		uint16_t tab[MAX_DEV_CNT];
		int phase;
		int sensNr;
	} zeroOfs;

	float getGasFactor(int id);
	bool execMyMenuItem(OutStream *strm, int idx, const char *cmd);
	char gtxt[200]; //używane przez "showMeas"
	void showMeas(OutStream *strm);
	void sendZeroOfs_FrameZero(uint16_t *tab);
	void sendZeroOfs_Phase1Frame();
	void sendZeroOfs_Phase3Frame();
	void sendZeroOfs_Phase5Frame();

protected:
	virtual void loopFunc();
	virtual void doOnTimeOut();

	virtual void onReciveData(bool replOK, uint8_t mdbFun, const uint8_t *tab, int regCnt);
	virtual void showState(OutStream *strm);
	virtual bool execMenuItem(OutStream *strm, int idx, const char *cmd);
	virtual const ShellItem* getMenu();
	virtual const char* getMenuName();

	const char* getSenName(int typ, int verTyp);
	const char* getSensValidStr(uint16_t status);
	bool isMeasValid(uint16_t status);

public:
	AirDetRs(int mdbNr, int portNr);
	bool getGasValue(MeasType measType, float *val);
	bool getGasValue(MeasType measType, int filtrType, float *val);
	bool isCfgAnyGas();
	bool isError();
	bool zeroGasFromSMS(const char *ptr, char *resText, int maxLen);
public:
	virtual void getDeviceStatusTxt(char *txt, int max);
};

#endif /* AIRDETRS_H_ */
