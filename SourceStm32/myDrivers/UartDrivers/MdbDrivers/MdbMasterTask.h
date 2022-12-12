/*
 * MdbMaster.h
 *
 *  Created on: 6 lut 2021
 *      Author: Grzegorz
 */

#ifndef MDBMASTERTASK_H_
#define MDBMASTERTASK_H_

#include <IOStream.h>
#include "UniDev.h"
#include "TaskClass.h"
#include "uart.h"
#include "GlobData.h"
#include "ShellItem.h"
#include "Filters.h"


//-----------------------------------------------------------------------------------------------------------------
// MdbUart
//-----------------------------------------------------------------------------------------------------------------
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

//-----------------------------------------------------------------------------------------------------------------
// MdbDev
//-----------------------------------------------------------------------------------------------------------------
typedef enum {
	reqEMPTY = 0, //
	reqCONSOLA, //
	reqSYS,
} ReqSrc;

class MdbMasterTask;
class MdbDev : public UniDev {
	friend class MdbMasterTask;

protected:
	MdbMasterTask *mMdb;
	uint8_t mMdbAdr;

	virtual void loopFunc()=0;
	virtual void onTimeOut()=0;
	virtual void onReciveData(bool replOK, uint8_t mdbFun, const uint8_t *tab, int regCnt)=0;
protected:
	virtual void showState(OutStream *strm)=0;
	virtual void shell(OutStream *strm, const char *cmd);
protected:
	uint16_t GetWord(const uint8_t *p);
	float GetFloat(const uint8_t *p);
	uint8_t getMdbNr();
	uint32_t getSentTick();
	bool isCurrenReq();
	void sendMdbFun3(ReqSrc reqSrc, uint8_t DevNr, uint16_t regAdr, uint16_t regCnt);
	void sendMdbFun4(ReqSrc reqSrc, uint8_t DevNr, uint16_t regAdr, uint16_t regCnt);
	void sendMdbFun6(ReqSrc reqSrc, uint8_t DevNr, uint16_t regAdr, uint16_t regVal);
	void sendMdbFun16(ReqSrc reqSrc, uint8_t DevNr, uint16_t regAdr, uint16_t regCnt, uint16_t *regTab);

public:
	MdbDev(MdbMasterTask *mdbTask, uint8_t mdbAdr, const char *name);
};

//-----------------------------------------------------------------------------------------------------------------
// MdbMasterTask
//-----------------------------------------------------------------------------------------------------------------
typedef struct {
	bool err;
	bool info;
	bool dat;
} DbgV;

#define MAX_VAL_CNT  10  // maksymalna ilość rejestrów dla funcji 16

typedef struct {
	uint8_t devNr;
	uint8_t fun;
	uint16_t regAdr;
	uint16_t regCnt;
	uint16_t regVal[MAX_VAL_CNT];
} ReqConsola;


class MdbMasterTask: public TaskClass {
	friend class MdbDev;
public:
	enum {
		MDB_1 = 1, MDB_2 = 2, MDB_3 = 3,
	};

	enum {
		SIGNAL_RXCHAR = 0x01, //
		SIGNAL_CMD = 0x02, //
		MAX_DEV = 4, //
	};
private:
	struct {
		int cnt;
		MdbDev *tab[MAX_DEV];
	} devs;
	struct {
		ShellItemFx *menuTab;
		void **argTab;
	} menuExp;

	enum {
		MAX_MDB_REPL_TM = 1000, // maksymalny czas na odpowiedz -> 1[sek]
	};
	MdbUart *mUart;

	uint8_t sendBuf[248];

	void sendMdbFun(ReqSrc reqSrc, uint8_t *buf, int len);
	void SetWord(uint8_t *p, uint16_t w);
	void proceedRecFrame();
	void sendConsolaReq();
	void setConsolaReq(ReqConsola *req);
	void doOnReciveData(bool replOK, uint8_t mdbFun, const uint8_t *tab, int regCnt);
	void doOnTimeOut();
	void addDev(MdbDev *dev);

protected:

	void buidDbgRec(DbgV *m);

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

	ReqConsola reqConsola;

protected:
	virtual void ThreadFunc();

	void showState(OutStream *strm);
	const char* getMenuName();
	const ShellItemFx* getMenuFx() {
		return NULL;
	}

public:
	MdbMasterTask(int mdbNr, int portNr);
	static void funDbgLevel(OutStream *strm, const char *cmd, void *arg);
	static void funShowState(OutStream *strm, const char *cmd, void *arg);
	static void funOnOffPower(OutStream *strm, const char *cmd, void *arg);
	static void funRdReg(OutStream *strm, const char *cmd, void *arg);
	static void funRdAnalogInp(OutStream *strm, const char *cmd, void *arg);
	static void funWdReg(OutStream *strm, const char *cmd, void *arg);
	static void funWrMulReg(OutStream *strm, const char *cmd, void *arg);
	static void funForChild(OutStream *strm, const char *cmd, void *arg);

	void Start(int BaudRate, int parity);
	void shell(OutStream *strm, const char *cmd);
	void setPower(bool pwr);
	bool getPower();
	bool getPowerFlt();
	int getTimeOutCnt() {
		return state.timeOutCnt;
	}

	bool isAnyConfiguredData();
	bool isDataError();
};

#endif
