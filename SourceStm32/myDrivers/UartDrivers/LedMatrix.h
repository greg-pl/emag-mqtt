/*
 * LedMatrix.h
 *
 *  Created on: 3 lip 2022
 *      Author: Grzegorz
 */

#ifndef LEDMATRIX_H_
#define LEDMATRIX_H_

#if (LED_MATRIX)

#include <uart.h>
#include <IOStream.h>


class LedMatrix: public TUart {
private:
	enum {
		PREAMBLE = 0x55, //
		POSTAMBLE = 0xAA, //

		MAX_SND_DT_LEN = 230, //
		MAX_REC_DT_LEN = 230, //

		// komendy wysyłane do panelu LED
		CMD_GET_INFO = 0x10, //
		CMD_SET_LIGHT_LEVEL = 0x11, //
		CMD_SELECT_GRAPHIC = 0x12, //
		CMD_UPLOAD_USER_GRAPHIC = 0x13, //

		// statusy zwracane przez panel LED
		STATUS_CMD_OK = 0x80, //
		STATUS_CMD_UKOWN = 0x81, //
		STATUS_CMD_NOT_COMPLETE = 0x82, //

		TM_AUTO_SEND = 2000, //
		TM_AUTO_TRY = 500, // czas pomiędzy kolejnymi próbami wysłania
		TM_MAX_REP = 200, // czas na przysłanie odpowiedzi
		TM_REQ_DELAY = 40, //
	};

	struct {
		uint8_t mRecByte; //
		int rxPtr; //
		uint32_t rxTick;
		uint8_t buf[MAX_REC_DT_LEN];

	} rxRec;

	struct {
		uint8_t buf[MAX_REC_DT_LEN];
	} txRec;

	struct {
		int loopCnt;
		int txCmplCnt;
		int rxCnt; //
		int recFrameCnt;
		int recFrameOkCnt;
		struct {
			int state;
			uint32_t stateTick;  // czas trwania w danym stanie
			uint32_t lastSendTick;
			uint32_t tryTick;
			int sendCnt; // licznik poprawnych przesłań
			int faceNr;
		} autoSend;
	} state;

	void showState(OutStream *strm);
	void startRecive();
	void execNewFrame();
	void sendFrame(const uint8_t *dt, int len);
	uint8_t makeCrc(const uint8_t *dt, int len);
	void sendCommand(uint8_t cmd);
	void sendCommand(uint8_t cmd, uint8_t param);
	void showRecDevInfo(uint8_t *dt);
	int getFaceNr(int prevNr, float pm2);
	void setState(int newState);
	int getStateTm();
	void ClrState();

protected:
	virtual void TxCpltCallback();
	virtual void RxCpltCallback();
	virtual void ErrorCallback();

public:
	LedMatrix(int PortNr);
	static void funShowState(OutStream *strm, const char *cmd, void *arg);
	static void funClearState(OutStream *strm, const char *cmd, void *arg);
	static void funGetInfo(OutStream *strm, const char *cmd, void *arg);
	static void funSetLight(OutStream *strm, const char *cmd, void *arg);
	static void funSetGraf(OutStream *strm, const char *cmd, void *arg);
	static void funTestFace(OutStream *strm, const char *cmd, void *arg);

	HAL_StatusTypeDef Init();

	void tick();
	void shell(OutStream *strm, const char *cmd);
};

#endif

#endif /* LEDMATRIX_H_ */
