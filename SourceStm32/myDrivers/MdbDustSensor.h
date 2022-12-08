/*
 * MdbDustSensor.h
 *
 *  Created on: 8 gru 2022
 *      Author: Grzegorz
 */

#ifndef MDBDUSTSENSOR_H_
#define MDBDUSTSENSOR_H_

#include "MdbMasterTask.h"

class MdbMasterDustTask: public MdbMasterTask {
private:
	enum{
		TM_AUTO_RD = 2000, // czas automatycznego odczytu czujnika hałasu
		MAX_TIME_REPL = 1500, // maksymalny czas odpowiedzi
		TIME_MEAS_VALID = 5000, //
		HEATER_CONST_ON = 0xA55A, //
	};
	struct {
		uint16_t devID; // ilość odczytana
		int  serialNumer; //ilośc odczytana ograniczona do MAX_DEV_CNT
		int  productYear;
		struct{
			int ver;
			int rev;
		}firmware;
		int failureCode;
		float temperature;
		bool heaterOn;
		int PmStatus;
		float pm1_0;
		float pm2_5;
		float pm4_0;
		float pm10;
		float dustCnt0_5;
		float dustCnt1_0;
		float dustCnt2_5;
		float dustCnt4_0;
		float dustCnt10;
		float dustSize;
	} dustData;

	struct {
		int phase;
		uint32_t tick;
		uint32_t redTick;
		int redCnt; // licznik odpowiedzi o pomiary
		int reqCnt; // licznik zapytań o pomiary
		char statusTxt[20];
		uint32_t heaterOrderLastSendTick;
	} autoRd;



	void showMeas(OutStream *strm);
	bool execMyMenuItem(OutStream *strm, int idx, const char *cmd);
	bool isMeasValid();

protected:
	virtual void loopFunc();
	virtual void doOnTimeOut();

	virtual void onReciveData(bool replOK, uint8_t mdbFun, const uint8_t *tab, int regCnt);
	virtual void showState(OutStream *strm);
	virtual bool execMenuItem(OutStream *strm, int idx, const char *cmd);
	virtual const ShellItem* getMenu();
	virtual const char* getMenuName();

public:
	MdbMasterDustTask(int mdbNr, int portNr);
	HAL_StatusTypeDef getMeas(DustMeasRec *meas);
	void setHeater(ReqSrc reqSrc, bool heaterOn);
	bool isCfgDustOn();
	bool isError();

};

#endif /* MDBDUSTSENSOR_H_ */
