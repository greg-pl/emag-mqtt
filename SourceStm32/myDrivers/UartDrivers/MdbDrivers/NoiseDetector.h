/*
 * NoiseDetector.h
 *
 *  Created on: 8 gru 2022
 *      Author: Grzegorz
 */

#ifndef NOISEDETECTOR_H_
#define NOISEDETECTOR_H_

#include <MdbMasterTask.h>

#if (DEV_NOISE)

class NoiseDetector: public MdbDev {
private:

	enum {
		TM_AUTO_RD = 2000, // czas automatycznego odczytu czujnika hałasu
		MAX_TIME_REPL = 1500, // maksymalny czas odpowiedzi
		TIME_MEAS_VALID = 5000, //
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
		int valueHd;
		float valueFiz;
		FiltrFIR *filtrFIR;
		FiltrIR *filtrIR;
	} noiseData;

	char gtxt[200]; //używane przez "showMeas"
	void showMeas(OutStream *strm);

protected:
	virtual void loopFunc();
	virtual void onTimeOut();
	virtual void onReciveData(bool replOK, uint8_t mdbFun, const uint8_t *tab, int regCnt);

	virtual void showState(OutStream *strm);
	virtual void shell(OutStream *strm, const char *cmd);
public:
	NoiseDetector(MdbMasterTask *mdbTask, uint8_t mdbAdr, const char *name);
	static void funShowMeasure(OutStream *strm, const char *cmd, void *arg);
public:
	//Unidev
	virtual bool getMeasValue(MeasType measType, float *val);
	virtual bool isMeasServiced(MeasType measType);
	virtual bool isDataError();
	//virtual void getDeviceStatusTxt(char *txt, int max);

};

#endif

#endif /* NOISEDETECTOR_H_ */
