/*
 * LedMatrix.cpp
 *
 *  Created on: 3 lip 2022
 *      Author: Grzegorz
 */

#include <LedMatrix.h>
#include <utils.h>
#include <string.h>
#include <UMain.h>
#include <GlobData.h>
#include <math.h>

typedef struct {
	uint8_t device; // wersja urządzenia
	uint8_t device_subtype; // podwersja urządzenia
	uint8_t firmware_version_aa; // główny numer wersji oprogramowania aa.bb (BCD)
	uint8_t firmware_version_bb; // dodatkowy numer wersji oprogramowania aa.bb (BCD)
	uint8_t resolution_x; // rozdzielczość wyświetlacza w poziomie (pixele)
	uint8_t resolution_y; // rozdzielczość wyświetlacza w pionie (pixele)
	uint8_t status; // kod awarii (0x00 – brak awarii)
} MatrixInfo;

LedMatrix::LedMatrix(int PortNr) :
		TUart::TUart(PortNr, 7) {
	memset(&state, 0, sizeof(state));
	memset(&rxRec, 0, sizeof(rxRec));
	memset(&txRec, 0, sizeof(txRec));

}

void LedMatrix::startRecive() {
	HAL_UART_Receive_IT(&mHuart, &rxRec.mRecByte, 1);
}

HAL_StatusTypeDef LedMatrix::Init() {
	HAL_StatusTypeDef st = TUart::Init(9600, parityEVEN);
	if (st == HAL_OK) {
		startRecive();
	}
	return st;
}

void LedMatrix::ErrorCallback() {

}

void LedMatrix::RxCpltCallback() {
	state.rxCnt++;
	if (rxRec.rxPtr < (int) sizeof(rxRec.buf)) {
		rxRec.buf[rxRec.rxPtr++] = rxRec.mRecByte;
		rxRec.rxTick = HAL_GetTick();
	}
	startRecive();
}

void LedMatrix::TxCpltCallback() {
	TUart::TxCpltCallback();
	state.txCmplCnt++;
}

uint8_t LedMatrix::makeCrc(const uint8_t *dt, int len) {
	uint8_t crc = 0x00;

	for (int j = 0; j < len; j++) {
		crc ^= dt[j];
		for (int ii = 0; ii < 8; ii++) {
			if (crc & 0x01)
				crc = (crc >> 1) ^ 0x8C;
			else
				crc >>= 1;
		}
	}
	return crc;
}

void LedMatrix::sendFrame(const uint8_t *dt, int len) {
	txRec.buf[0] = PREAMBLE;
	txRec.buf[1] = len;
	memcpy(&txRec.buf[2], dt, len);
	txRec.buf[2 + len] = makeCrc(dt, len);
	txRec.buf[3 + len] = POSTAMBLE;
	writeBuf(txRec.buf, 4 + len);
}

void LedMatrix::sendCommand(uint8_t cmd) {
	sendFrame(&cmd, 1);
}

void LedMatrix::sendCommand(uint8_t cmd, uint8_t param) {
	uint8_t buf[2];
	buf[0] = cmd;
	buf[1] = param;
	sendFrame(buf, 2);
}

void LedMatrix::showRecDevInfo(uint8_t *dt) {
	MatrixInfo *inf = (MatrixInfo*) dt;
	MsgStream *strm = getOutStream();
	if (strm->msgOpen(colWHITE)) {
		strm->msgItem("Device code   :0x%02X", inf->device);
		strm->msgItem("Device subcode:0x%02X", inf->device_subtype);
		strm->msgItem("Firm.Ver      :%u.%03u", inf->firmware_version_aa, inf->firmware_version_bb);
		strm->msgItem("Resolution    :%u x %u", inf->resolution_x, inf->resolution_y);
		strm->msgItem("Status        :%u", inf->status);
		strm->msgClose();
	}
}

void LedMatrix::execNewFrame() {

	state.recFrameCnt++;
	if (rxRec.buf[0] == PREAMBLE && rxRec.buf[rxRec.rxPtr - 1] == POSTAMBLE) {
		uint8_t len = rxRec.buf[1];
		if (len + 4 == rxRec.rxPtr) {
			uint8_t nCrc = makeCrc(&rxRec.buf[2], len);
			if (nCrc == rxRec.buf[len + 2]) {
				state.recFrameOkCnt++;

				uint8_t *rData = &rxRec.buf[2];
				uint8_t rCmd = rData[0];
				uint8_t rStatus = rData[1];
				if (rStatus == STATUS_CMD_OK) {

					switch (rCmd) {
					case CMD_GET_INFO:
						showRecDevInfo(&rData[2]);
						break;
					case CMD_SELECT_GRAPHIC:
						if (state.autoSend.state == 2)
							setState(3);
						break;
					case CMD_SET_LIGHT_LEVEL:
						if (state.autoSend.state == 5)
							setState(6);
						break;
					default:
						getOutStream()->msg(colRED, "Matrix unknow cmd : 0x%02X", rCmd);
						break;
					}
				}
			}
		}
	}
}

enum {
	face_empty = 0x00, //
	face_custom = 0x01, //
	face_grin_beam = 0x02, //
	face_grin = 0x03, //
	face_smile = 0x04, //
	face_meh = 0x05, //
	face_frown_open = 0x06, //
	face_angry = 0x07, //
};

int LedMatrix::getFaceNr(int prevNr, float pm2) {
	int res;

	if (isnanf(pm2)) {
		res = face_empty;
	} else {
		res = face_grin_beam;
		prevNr -= (1 + face_grin_beam);

		for (int i = 0; i < FACE_LIMIT_TAB_LEN; i++) {
			float lim = config->data.R.rest.faceLimitTab[i];
			if (i <= prevNr)
				lim -= config->data.R.rest.faceHistereza;
			if (pm2 > lim) {
				res = i + 1 + face_grin_beam;
			}
		}
	}
	return res;
}

void LedMatrix::setState(int newState) {
	state.autoSend.state = newState;
	state.autoSend.stateTick = HAL_GetTick();
}

int LedMatrix::getStateTm() {
	return HAL_GetTick() - state.autoSend.stateTick;
}

void LedMatrix::tick() {
	state.loopCnt++;
	uint32_t tt = HAL_GetTick();

	if (rxRec.rxPtr > 0 && tt - rxRec.rxTick > 10) {
		execNewFrame();
		rxRec.rxPtr = 0;
	}
	if (config->data.R.rest.faceAutoSend) {

		switch (state.autoSend.state) {
		case 0:
			if (HAL_GetTick() - state.autoSend.lastSendTick > TM_AUTO_SEND) {
				if (HAL_GetTick() - state.autoSend.tryTick > TM_AUTO_TRY) {
					state.autoSend.tryTick = HAL_GetTick();

					HAL_StatusTypeDef st = GlobData::getDustMeas(&state.dustMeas);
					if (st == HAL_OK) {
						state.autoSend.faceNr = getFaceNr(state.autoSend.faceNr, state.dustMeas.pm2_5);
						setState(1);
					}
				}
			}

			break;
		case 1:
			sendCommand(CMD_SELECT_GRAPHIC, state.autoSend.faceNr);
			setState(2);
			break;

		case 2:
			//czekaie na odpowiedź
			if (getStateTm() >= TM_MAX_REP)
				setState(0);
			break;

		case 3:
			// opóźnienie wysłania drugiego zapytania
			if (getStateTm() >= TM_REQ_DELAY)
				setState(4);
			break;

		case 4:
			sendCommand(CMD_SET_LIGHT_LEVEL, config->data.R.rest.faceLevel);
			setState(5);
			break;

		case 5:
			//czekaie na odpowiedź
			if (getStateTm() >= TM_MAX_REP)
				setState(0);
			break;
		case 6:
			setState(0);
			state.autoSend.lastSendTick = HAL_GetTick();
			state.autoSend.sendCnt++;

			break;

		}

	}

}

void LedMatrix::showState(MsgStream *strm) {
	if (strm->msgOpen(colWHITE)) {
		strm->msgItem("___LedMatrix Status___");

		strm->msgItem("loopCnt=%u", state.loopCnt);
		strm->msgItem("txCmplCnt=%u", state.txCmplCnt);
		strm->msgItem("rxCnt=%u", state.rxCnt);
		strm->msgItem("IrqCnt=%u", mIrqCnt);
		strm->msgItem("recFrameCnt=%u", state.recFrameCnt);
		strm->msgItem("recFrameOkCnt=%u", state.recFrameOkCnt);
		strm->msgItem("sendAutoCntOk=%u", state.autoSend.sendCnt);
		strm->msgItem("sendAutoFaceNr=%u", state.autoSend.faceNr);

		strm->msgClose();
	}
}

const ShellItem menuLedMatrix[] = { //
		{ "s", "stan" }, //
				{ "clrCnt", "zeruj liczniki statystyki" }, //
				{ "getInfo", "Pobranie info z panelu" }, //
				{ "setLevel", "ustawienie jasności" }, //
				{ "setGraph", "ustawienie grafiki" }, //
				{ "testFace", "testuj pobranie numeru buźki: testFace prev_lev pm_val" }, //

				{ NULL, NULL } };

void LedMatrix::shell(MsgStream *strm, const char *cmd) {
	char tok[20];
	int idx = -1;

	if (Token::get(&cmd, tok, sizeof(tok)))
		idx = findCmd(menuLedMatrix, tok);
	switch (idx) {
	case 0: //s
		showState(strm);
		break;
	case 1: //clrCnt
		state.loopCnt = 0;
		mIrqCnt = 0;
		state.txCmplCnt = 0;
		state.rxCnt = 0;
		state.recFrameCnt = 0;
		state.recFrameOkCnt = 0;
		break;
	case 2: //getInfo
		strm->msg(colWHITE, "sendGetInfo");
		sendCommand(CMD_GET_INFO);
		break;

	case 3: { //setLevel
		int lev;
		if (Token::getAsInt(&cmd, &lev)) {
			sendCommand(CMD_SET_LIGHT_LEVEL, lev);
			strm->msg(colWHITE, "sendLight %u", lev);
		}
	}
		break;
	case 4: { //setGraph
		int graph;
		if (Token::getAsInt(&cmd, &graph)) {
			sendCommand(CMD_SELECT_GRAPHIC, graph);
			strm->msg(colWHITE, "setGraph %u", graph);
		}
	}
		break;
	case 5: { //testFace
		float pm;
		int prev;
		if (Token::getAsInt(&cmd, &prev)) {
			if (Token::getAsFloat(&cmd, &pm)) {
				int faceNr = getFaceNr(prev, pm);
				strm->msg(colWHITE, "prev=%u  pm=%.2f[%%] faceNr=%u", prev, pm, faceNr);
			}
		}
	}
		break;

	default:
		showHelp(strm, "LedMatrix Menu", menuLedMatrix);
		break;
	}

}
