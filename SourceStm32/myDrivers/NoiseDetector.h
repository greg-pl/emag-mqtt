/*
 * NoiseDetector.h
 *
 *  Created on: 8 gru 2022
 *      Author: Grzegorz
 */

#ifndef NOISEDETECTOR_H_
#define NOISEDETECTOR_H_

#include <MdbMasterTask.h>

class NoiseDetector: public MdbMasterTask {
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
		int valueHd; //[ppb]
		float valueFiz; //[ug/m3]
		FiltrFIR *filtrFIR;
		FiltrIR *filtrIR;
	} noiseData;

	char gtxt[200]; //używane przez "showMeas"
	void showMeas(OutStream *strm);

protected:
	virtual void loopFunc();
	virtual void doOnTimeOut();

	virtual void onReciveData(bool replOK, uint8_t mdbFun, const uint8_t *tab, int regCnt);
	virtual void showState(OutStream *strm);
	virtual const char* getMenuName();
	virtual const char* getDevName();
	virtual const ShellItemFx* getMenuFx();

public:
	NoiseDetector(int mdbNr, int portNr);
	static void funShowMeasure(OutStream *strm, const char *cmd, void *arg);

	bool getNoiseValue(float *val);
	bool getNoiseValue(int filtrType, float *val);
	bool isCfgNoiseOn();
	bool isError();

};


#endif /* NOISEDETECTOR_H_ */
