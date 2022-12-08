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
#include "Filters.h"

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
public:
	virtual bool isAnyConfiguredData(){
		return false;
	}
	virtual bool isDataError(){
		return false;
	}
	virtual bool getMeasValue(MeasType measType, float *val){
		return false;
	}
	virtual bool getMeasValue(MeasType measType, int filtrType, float *val){
		return false;
	}
	virtual void getDeviceStatusTxt(char *txt, int max){

	}
};




#endif /* MDBMASTERTASK_H_ */
