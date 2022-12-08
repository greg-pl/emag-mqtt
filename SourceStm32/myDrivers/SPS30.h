/*
 * SPS30.h
 *
 *  Created on: 17 mar 2021
 *      Author: Grzegorz
 */

#ifndef SPS30_H_
#define SPS30_H_

#include <DustSensorBase.h>
#include <uart.h>

//rekord danych odczytywany z SPS30
typedef union {
	uint32_t tab[10];
	struct {
		float PM_1_0;  // Mass Concentration PM1.0
		float PM_2_5;	//PM2.5
		float PM_4_0;
		float PM_10;
		float NUM_0_5; //Number Concentration
		float NUM_1_0;
		float NUM_2_5;
		float NUM_4_0;
		float NUM_10;
		float ParticleSize;
	};
} SPS30_Data;

typedef enum {
	stOK = 0, //
	stERR_NO_START_BYTE, //
	stERR_SUM, //
	stFRAME_TOO_BIG, //
} SPS30_Status;

class SPS30: public DustSensorBase, public TUart {

private:
	//budowa ramki SPS30
	enum {
		START_BYTE = 0x7E, //
		SLAVE_ADR = 0, //
	};
	//komendy rozumiane przez SPS30
	enum {
		cmdSTART_MEASURE = 0, //
		cmdSTOP_MEASURE = 1, //
		cmdREAD_VAL = 3, //
		cmdSET_AUTOCLEANING_INTERVAL = 0x80, //
		cmdSTART_FAN_CLEANING = 0x56, //
		cmdDEVICE_INFO = 0xD0, //
		cmdRESET = 0xD3, //
	};

	enum {
		SPS30_MAX_SND_DT_LEN = 0x20, //
		SPS30_MAX_REC_DT_LEN = 0x20, //
		SPS30_SND_FRAME_SIZE = 4 + 2 * SPS30_MAX_SND_DT_LEN + 2, //
		SPS30_REC_FRAME_SIZE = 4 + 2 * SPS30_MAX_REC_DT_LEN + 2, //
	};

	struct {
		uint8_t mRecByte; //
		int rxPtr; //
		uint8_t buf[SPS30_REC_FRAME_SIZE];
		bool frameComplete;
	} rxRec;

	struct {
		int txLen;
		bool mSending; //
		int inPtr; //
		uint8_t buf[SPS30_SND_FRAME_SIZE];
		dword lastSendTick;
	} txRec;

	struct {
		int txCmplCnt;
		int rxCnt; //
		uint32_t powerOnTick;
		int recivedFrameCnt;
		bool isMeasOn;
		uint8_t devState;  //stan urządzenia, odczytany z ramki SPS30
		struct {
			uint32_t startMeasureTick; // kiedy wys�ano rozkaz uruchomienia pomiar�w
			bool startMeasureSended;
		} measStart;

		struct {
			uint32_t getMeasTick;  // czas odebrania danych
			uint32_t sendGetReqMeasTick;  //czas wys�ania rozkazu odczytu
		} measFrame;

	} state;

	SignaledClass *mSignObj;

	SPS30_Data measSps30;
	int mShowMeasCnt;
	int mDebug;

	void putTxFrame(uint8_t a);
	void SendFrame(uint8_t cmd, const uint8_t *dt, uint8_t len);
	void ShowMesuredRec(OutStream *strm);

	SPS30_Status execNewFrame();
	void sendGetDevInfo(uint8_t par);
	void sendDoReset();
	void sendStartMeasure();
	void sendStopMeasure();
	void sendGetMeasure();
	void sendRunCleaning();

protected:
	virtual void TxCpltCallback();
	virtual void RxCpltCallback();
	virtual void ErrorCallback();
public:
	SPS30();
	virtual void StartMeas();
	virtual void StopMeas();
	virtual void shell(OutStream *strm, const char *cmd);
	virtual void setPower(bool on);
	virtual HAL_StatusTypeDef Init(SignaledClass *signObj);
	virtual void tick();

};

#endif /* SPS30_H_ */
