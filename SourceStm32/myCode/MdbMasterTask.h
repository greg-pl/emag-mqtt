/*
 * MdbMaster.h
 *
 *  Created on: 6 lut 2021
 *      Author: Grzegorz
 */

#ifndef MDBMASTERTASK_H_
#define MDBMASTERTASK_H_

#include <IOStream.h>
#include "TaskClass.h"
#include "uart.h"
#include "GlobData.h"
#include "DustSensorBase.h"
#include "ShellItem.h"

typedef enum {
	reqEMPTY = 0, //
	reqCONSOLA, //
	reqSYS,
} ReqSrc;

class MdbUart: public TUart {
public:
	enum {
		FRAME_LEN = 240,
	};
private:
	osThreadId mThreadId;

	void setTxEn(bool txON);
	struct {
		int globCnt;
		int ptr;
		uint8_t buf[FRAME_LEN + 1];
		uint32_t tick;
	} rxRec;

protected:
	virtual void TxCpltCallback();
	virtual void RxCpltCallback();
public:
	MdbUart(int PortNr, int Priority);
	HAL_StatusTypeDef Init(int BaudRate, int parity);
	void writeBuf(const void *buf, int len);
	void setThreadId(osThreadId threadId) {
		mThreadId = threadId;
	}
	uint32_t getLastRxCharTick() {
		return rxRec.tick;
	}
	int getRxCharCnt() {
		return rxRec.ptr;
	}
	int getRxGlobCnt() {
		return rxRec.globCnt;
	}
	const uint8_t* getRxBuf() {
		return rxRec.buf;
	}
	void clearRxBuf();
};

typedef struct {
	bool err;
	bool info;
	bool dat;
} MsgV;


class MdbMasterTask: public TaskClass {
public:
	enum {
		MDB_1 = 1, MDB_2 = 2, MDB_3 = 3,
	};

	enum {
		SIGNAL_RXCHAR = 0x01, //
		SIGNAL_CMD = 0x02, //
	};
private:
	enum {
		MAX_VAL_CNT = 10, // maksymalna ilość rejestrów dla funcji 16
		MAX_MDB_REPL_TM = 1000, // maksymalny czas na odpowiedz -> 1[sek]

	};
	MdbUart *mUart;

	uint8_t sendBuf[248];

	void sendMdbFun(ReqSrc reqSrc, uint8_t *buf, int len);
	void SetWord(uint8_t *p, uint16_t w);
	void proceedRecFrame();
	void sendConsolaReq();

protected:
	struct {
		ShellItem *tab;
		int baseCnt;
	} menu;
	void buildMenu(const ShellItem *toAddMenu);
	void buidMsgRec(MsgV *m);

protected:
	uint16_t GetWord(const uint8_t *p);
	float GetFloat(const uint8_t *p);
	void sendMdbFun3(ReqSrc reqSrc, uint8_t DevNr, uint16_t regAdr, uint16_t regCnt);
	void sendMdbFun4(ReqSrc reqSrc, uint8_t DevNr, uint16_t regAdr, uint16_t regCnt);
	void sendMdbFun6(ReqSrc reqSrc, uint8_t DevNr, uint16_t regAdr, uint16_t regVal);
	void sendMdbFun16(ReqSrc reqSrc, uint8_t DevNr, uint16_t regAdr, uint16_t regCnt, uint16_t *regTab);

	int mMdbNr;
	int mDbgLevel;
	struct {
		struct {
			ReqSrc currReq; // przetwarzane żadanie modbus
			uint8_t devNr;
			uint8_t code;
			uint16_t regAdr;
			uint16_t regCnt;
			uint16_t regVal;
			uint32_t tick;
		} sent;
		int timeOutCnt;
	} state;

	struct {
		uint8_t devNr;
		uint8_t fun;
		uint16_t regAdr;
		uint16_t regCnt;
		uint16_t regVal[MAX_VAL_CNT];
	} reqConsola;

protected:
	virtual void ThreadFunc();
	virtual void loopFunc() {
	}
	virtual void doOnTimeOut() {

	}

	virtual void showState(OutStream *strm);
	virtual void onReciveData(bool replOK, uint8_t mdbFun, const uint8_t *tab, int regCnt) {
	}
	virtual bool execMenuItem(OutStream *strm, int idx, const char *cmd);
	virtual const ShellItem* getMenu();
	virtual const char* getMenuName();

public:
	MdbMasterTask(int mdbNr, int portNr);
	void Start(int BaudRate, int parity);
	void shell(OutStream *strm, const char *cmd);
	void setPower(bool pwr);
	bool getPower();
	bool getPowerFlt();

};

class FiltrIR {
private:
	bool firstDt;
	float state;
	float mK;
public:
	FiltrIR(float k);
	void inp(float x);
	float out();
	void setK(float K) {
		mK = K;
	}
};

class FiltrFIR {
private:
	enum {
		MAX_LEN = 120, //
	};
	float tab[MAX_LEN];
	bool mOverride; // czy pamięc filtru już się przewinęła
	int mPtr;
	int mLen; // długość filtru
public:
	FiltrFIR(int len);
	void inp(float x);
	float out();
};

typedef struct {
	int Typ;
	int VerTyp;
	int Status;
	int valueHd; //[ppb]
	float valueFiz; //[ug/m3]
	FiltrFIR *filtrFIR;
	FiltrIR *filtrIR;
} SensorData;

class MdbMasterGasTask: public MdbMasterTask {
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
	MdbMasterGasTask(int mdbNr, int portNr);
	bool getGasValue(MeasType measType, float *val);
	bool getGasValue(MeasType measType, int filtrType, float *val);
	void getKomoraStatusTxt(char *txt, int max);
	bool isCfgAnyGas();
	bool isError();
	bool zeroGasFromSMS(const char *ptr, char *resText, int maxLen);
};

class MdbMasterNoiseTask: public MdbMasterTask {
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

	bool execMyMenuItem(OutStream *strm, int idx, const char *cmd);
	char gtxt[200]; //używane przez "showMeas"
	void showMeas(OutStream *strm);

protected:
	virtual void loopFunc();
	virtual void doOnTimeOut();

	virtual void onReciveData(bool replOK, uint8_t mdbFun, const uint8_t *tab, int regCnt);
	virtual void showState(OutStream *strm);
	virtual bool execMenuItem(OutStream *strm, int idx, const char *cmd);
	virtual const ShellItem* getMenu();
	virtual const char* getMenuName();

public:
	MdbMasterNoiseTask(int mdbNr, int portNr);
	bool getNoiseValue(float *val);
	bool getNoiseValue(int filtrType, float *val);
	bool isCfgNoiseOn();
	bool isError();

};

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

#endif /* MDBMASTERTASK_H_ */
