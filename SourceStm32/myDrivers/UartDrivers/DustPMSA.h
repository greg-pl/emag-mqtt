/*
 * DustPMSA.h
 *
 *  Created on: Dec 7, 2020
 *      Author: Grzegorz
 */

#ifndef DUSTPMSA_H_
#define DUSTPMSA_H_

#include <DustSensorBase.h>
#include <uart.h>
#include "cmsis_os.h"

//rekord danych odczytywany z PMSA
typedef struct {
	float PM_1_0;  // Mass Concentration PM1.0
	float PM_2_5;	//PM2.5
	float PM_10;

	float PM_AE_1_0;  // Mass Concentration PM1.0
	float PM_AE_2_5;	//PM2.5
	float PM_AE_10;

	uint16_t NUM_D03;
	uint16_t NUM_D05;
	uint16_t NUM_D10;
	uint16_t NUM_D25;
	uint16_t NUM_D50;
	uint16_t NUM_D100;
	uint16_t Reserved;
	// tylko PMS5003ST
	float Formaldehyde; // mg/m3
	float Temper; // [stop C]
	float Humidity; // [%]
	uint8_t FirmwareVer; //
	uint8_t ErrorCode; //
} PMSA_Data;

class DustPMSA: public DustSensorBase, public TUart {
private:
	enum {
		PMSA_MAX_SND_DT_LEN = 0x20, //
		PMSA_MAX_REC_DT_LEN = 0x30, //
		PMSA_SND_FRAME_SIZE = 4 + 2 * PMSA_MAX_SND_DT_LEN + 2, //
		PMSA_REC_FRAME_SIZE = 4 + 2 * PMSA_MAX_REC_DT_LEN + 2, //
	};
	struct {
		uint8_t mRecByte; //
		int rxPtr; //
		uint32_t rxTick;
		uint8_t buf[PMSA_REC_FRAME_SIZE];
	} rxRec;

	struct {
		uint8_t buf[PMSA_SND_FRAME_SIZE];
		uint32_t lastSendTick;
	} txRec;

	SignaledClass *mSignObj;
	PMSA_Data measHpma;
	int mShowMeasCnt;
	bool mFormaldehydeExist;

	struct {
		int loopCnt;
		int txCmplCnt;
		int rxCnt; //
		uint32_t measureOnTick;
		uint32_t measureOffTick;
		uint32_t powerChgTick;
		int recFrameCnt;
		int recFrameSumOkCnt;
		int powerOffCnt;
		int recFrameLenErr;
		bool isMeasureOn;
		bool isPowerOn;

		bool ackFlag;
		bool ackResult;

		struct {
			uint32_t startMeasureTick; // kiedy wys�ano rozkaz uruchomienia pomiar�w
			bool startMeasureSended;
		} measStart;

		struct {
			uint32_t measRecivedTick;
			bool getMeasSended; // zapalana po wysłaniu rozkazu GetMeas, kasowana po debraniu danych
			uint32_t prevMeasTick;  // czas odebrania danych
			uint32_t getMeasTick;  // czas odebrania danych
			uint32_t sendGetReqMeasTick;  //czas wys�ania rozkazu odczytu
		} measFrame;

	} state;

	void SendCmd(uint8_t cmd, uint16_t arg);
	void showState(OutStream *strm);
	void clrState();

	void sendGetMeasure();
	void sendGoSleep();
	void sendWakeUp();
	void sendSetActiveMode();
	void sendSetPassiveMode();
	HAL_StatusTypeDef execNewFrame();
	void ShowMesuredRec();
	void startRecive();


protected:
	virtual void TxCpltCallback();
	virtual void RxCpltCallback();
	virtual void ErrorCallback();
	virtual bool isFormaldehyde(){
		return true;
	}

public:
	DustPMSA(bool formal_exist);

	static void funShowState(OutStream *strm, const char *cmd, void *arg);
	static void funSetPower(OutStream *strm, const char *cmd, void *arg);
	static void funGetMeas(OutStream *strm, const char *cmd, void *arg);
	static void funWakeup(OutStream *strm, const char *cmd, void *arg);
	static void funGoSleep(OutStream *strm, const char *cmd, void *arg);
	static void funSetActiveMode(OutStream *strm, const char *cmd, void *arg);
	static void funSetPassiveMode(OutStream *strm, const char *cmd, void *arg);
	static void funSetHdSleep(OutStream *strm, const char *cmd, void *arg);
	static void funShowNmMeas(OutStream *strm, const char *cmd, void *arg);
	static void funClrStat(OutStream *strm, const char *cmd, void *arg);

	virtual void StartMeas();
	virtual void StopMeas();
	virtual void shell(OutStream *strm, const char *cmd);
	virtual void setPower(bool on);
	virtual HAL_StatusTypeDef Init(SignaledClass *signObj);
	virtual void tick();

};

#endif /* DUSTPMSA_H_ */
