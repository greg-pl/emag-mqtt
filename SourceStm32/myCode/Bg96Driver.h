/*
 * Bg96Driver.h
 *
 *  Created on: Dec 16, 2020
 *      Author: Grzegorz
 */

#ifndef BG96DRIVER_H_
#define BG96DRIVER_H_

#include <IOStream.h>
#include "TaskClass.h"
#include "uart.h"
#include "myDef.h"

#define CTRL_Z "\032"
#define CTRL_Z_CH '\032'

enum {
	MAX_AT_LINE_SIZE = 128,
};

typedef enum {
	regERROR = 0, //
	regNO_REG, //
	regREG_HOME, //
	regREG_DENIED, //
	regREG_UNKNOWN, //
	regREG_ROAMING, //
	regREG_BADREPLAY, //
} RegStatus;

typedef enum {
	phSTOP = 0, //
	phSTART, //
	phWAIT_FOR_STATUS, //
	phWAIT_STATUS_OK,
	phFIRST_AT,
	phFIRST_AT_OK,
	phPARAMS_OK,
	phPIN_OK,
	phCREG_OK,
	phLOGGED_APN,

} BgPhase;

extern "C" const char* getPhaseName(BgPhase ph);

class Bg96Uart: private TUart {
private:
	osThreadId mThreadId;
	bool wiatForSent();
	RxTxBuf *inpStrem;
	char txBuf[MAX_AT_LINE_SIZE];
	char rxChar;
public:
	struct {
		int rxCnt;
		int lnCnt;
		int toBigLineCnt;
	} stat;

protected:
	//wywoływane z przerwania
	virtual void TxCpltCallback();
	virtual void RxCpltCallback();
public:
	Bg96Uart(int PortNr);
	HAL_StatusTypeDef Init();

	void setThreadId(osThreadId threadId) {
		mThreadId = threadId;
	}
	void writeBuf(const char *txt, int len);
	void writeStr(const char *txt, bool addNl);
	void writeStrNoCpy(const char *txt);
	bool readLn(char *buf, int max);
	void clearInpStream();
};

typedef struct {
	uint8_t sslversion;
	uint8_t seclevel;
	uint16_t ciphersuite;
	char cacert[40];
	char clientcert[40];
	char clientkey[40];
	bool sni;
	bool checkhost;
	bool ignorecertchain;
	bool ignorelocaltime;
	int negotiatetime;
	bool dtls;
	int dtlsversion;

} SSL_Cfg;

enum {
	ixEmptyLn = 0,  // pusta linia
	ixOK,  // odebrano linię OK
	ixRDY, // odebrano linię RDY
	ixError, //odebrano linię ERROR
	ixErrCME,  // "+CME ERROR:" kod błedu
	ixErrCMS,  // "+CMS ERROR:" kod błedu
	ixQmtStat,  // "+QMTSTAT:"
	ixQmtPub,  // "+QMTPUB:"
	ixQmtOpen,  // "+QMTOPEN:"
	ixQmtConn,  // "+QMTCONN:"
	ixQmtClose,  // "+QMTCLOSE:"
	ixQNtp,  // "+QNTP:"
	ixQSslCfg,  // "+QSSLCFG:"
	ixCsq,  // "+CSQ:"
	ixQGpsLoc,  // "+QGPSLOC:"
	ixCreg,  // "+CREG:"
	ixQiact,  // "+QIACT:
	ixCpin,  // "+CPIN:
	ixFOpen, //+QFOPEN:
	ixCMTI, //+CMTI:
	ixCMGR, //+CMGR:
	xQSimStat, //+QSIMSTAT:
	xCtze, //+CTZE: "+04",0,"2021/02/02,18:36:23"

	ixNoURC, // gdy nie zaszedł inny warunek
};

enum {

	nnEmptyLn = (1 << ixEmptyLn),
	nnOK = (1 << ixOK),
	nnRDY = (1 << ixRDY),
	nnError = (1 << ixError),
	nnErrCME = (1 << ixErrCME),
	nnErrCMS = (1 << ixErrCMS),
	nnQmtStat = (1 << ixQmtStat),
	nnQmtPub = (1 << ixQmtPub),
	nnQmtOpen = (1 << ixQmtOpen),
	nnQmtConn = (1 << ixQmtConn),
	nnQmtClose = (1 << ixQmtClose),
	nnQNtp = (1 << ixQNtp),
	nnQSslCfg = (1 << ixQSslCfg),
	nnCsq = (1 << ixCsq),
	nnQGpsLoc = (1 << ixQGpsLoc),
	nnCreg = (1 << ixCreg),
	nnQiact = (1 << ixQiact),
	nnCpin = (1 << ixCpin),
	nnFOpen = (1 << ixFOpen),
	nnCMTI = (1 << ixCMTI),
	nnCMGR = (1 << ixCMGR),
	nnQSimStat = (1<<xQSimStat),
	nnCtze = (1<<xCtze),
	nnNoURC = (1 << ixNoURC),
};

//rekord danych odebranych

typedef struct {
	uint32_t flags;
	char recBuf[200];
	int codeErrCME;
	int codeErrCMS;
	int fileNr;
	int wResult;
	int qmtErr;
	int qmtRepl;
	RegStatus regStatus;
	struct {
		int contextID;
		char paramName[20];
		char paramValue[60];
	} sslCfg;
	int smsIdx;
} ReciveRec;

class Bg96Driver: public TaskClass {
private:

	enum {
		stOK = 0, //
		stTIME_OUT = -1, //
		stUNKN_REPL = -2, //
		stBREAK = -3, //
		stNOT_LOGED = -4, //
		stREG_ERROR = -5, //
		stNO_DATA = -6, //  brak danych
		stSOFT_EXCP = -7, //  błąd programowy
		stBgError = -8, // BG96 wysłał komunikat ERROR
		stNO_SIM_CARD = -9, //nie włożona karta SIM

	};

	enum {
		GPS_ONE_TRY_MAX = 10, //
		GPS_ONE_TRY_TIME = 30 * 1000, //co jaki czas próbować odczytać GPS w trybie jednokrotnego odczytu
		GPS_TRY_MAX = 4, //
		GPS_TRY_TIME = 2 * 1000, // co jaki czas próbować odczytać GPS w trybie ciągłego oczytyu

	};
public:
	enum {
		SIGNAL_CHAR = 0x01, //
		SIGNAL_CHAR_NL = 0x02, //
		SIGNAL_TXCPL = 0x04, //
		SIGNAL_BREAK = 0x08, //
		SIGNAL_MSG = 0x10, //
	};

	enum {
		ERR_BASE_CME = 1000, //
		ERR_BASE_CMS = 2000, //
		ERR_BASE_QMTOPEN = 100, // AT+QMTOPEN
		ERR_BASE_QMTCONN = 200, // AT+QMTCONN
		ERR_BASE_QMTPUB = 300, // AT+QMTPUB

	};

private:
	Bg96Uart *uart;
	int loopCnt;
	BgPhase mPhase;
	struct {
		volatile bool doStop;
		volatile bool doStart;
		volatile bool doPowerUp;
		volatile bool mRunning;
		volatile bool doOpenMqtt;
		volatile bool doCloseMqtt;
		volatile bool doSendMqtt;
		volatile bool doGetNtpTime;
		volatile bool doGetGps;
		int sendMqqtIdx;
		int sendMqqtVal;
		int sslContext;
		volatile bool doShowSSlContext;
		volatile bool doSetSSlContext;
		volatile bool doRestart;

	} asynch;

	ReciveRec recR;
	char mTmpBuf[200];

	typedef enum {
		vvOFF = 0, vvERROR, vvWARN, vvHINT, vvTEXT,
	} LogLevel;
	struct {
		bool rx;
		bool tx;
		int logV;
		int timeMode;
	} mEcho;
	void logT(LogLevel lev, const char *frm, ...);

	uint32_t mStartTick;
	uint32_t mSendTick;

	void set3_8V(bool flag);
	bool get3_8V();
	void setReset(bool flag);
	bool getReset();
	void setPowerKey(bool flag);
	bool getPowerKey();
	bool getApReady();
	bool getHdStatus();
	bool getDCD();
	bool getRI();
	void setDTR(bool flag);
	bool getDTR();
private:
	bool setEchoMode(const char *cmd);
	void setEchoMode(OutStream *strm, const char *cmd);
	void showRxEcho(int idx, const char *txt);
	void zeroState();
	void clearRecDt();

	void showState(OutStream *strm);
	void showHdwState(OutStream *strm);
	void showInformation(OutStream *strm);
	void showGpsInformation(OutStream *strm);

	int cmpToArray(const char *txt, const char *const*array);

	void runLoop();
	void idleLoop();

	void powerUp();
	int initBg();
	int SendPIN();
	RegStatus checkNetworkRegisterStatus();
	int registerToNetwork();
	int checkAPNrdy();
	int enterAPN();

	void writeCmd(const char *cmd);
	void writeCmdF(const char *pFormat, ...);
	void writeCmdNoEcho(const char *cmd, bool doDataCpy);

	void clearBreakSignal();
	int dwb(uint32_t tm);

	void resolveLine();

	int getAnswLn(char *repl, int max, int time);
	int getReplUntil(uint32_t inpFlag, int time);
	int getReplUntil(int time);
	int getReplOk(int time);
	int getReplWithErr(uint32_t flag, int time);
	int openBGFile(const char *fileName, int openMode, int *fileHandle);
	int closeBGFile(int fileHandle);
	void execNewSMS();


	void parseCreg(const char *ptr);
	void parseNtpTime(const char *ptr);
	bool parseTimeStr(const char *ptr, TDATE *tm);
	void parseGpsData(const char *ptr);
	void parseNetworkTime(const char *ptr);
	void parseRSSI(const char *ptr);
	int getBgData(const char *cmd, char *buf, int max, int time);

	int openMqqtConnection();
	int closeMqqtConnection();
	int sendMqqtData(const char *varName, const char *data, bool doDataCpy);
	int sendMqqt(int idx, int val);
	int showSslContext(int contextNr);
	int setSslContext(int contextNr, const SSL_Cfg *pCfg);

	int turnOnGps();
	int reciveNtpTime();
	int reciveRSSI();
	int readGpsData();

	void doFlushInpBuffer();
	void rssiLoopFun();
	void gpsLoopFun();
	void ntpLoopFun();
	void mqttLoopFun();
	void smsLoopFun();
	void smsSendLoopFun();


protected:
	//TaskClass
	virtual void ThreadFunc();
public:
	struct {
		char model[20]; //BG96
		char firmVer[40]; //BG96MAR02A07M1G_01.017.01.017
		char imei[20];    //860536047966892
		bool file_0_server_exist;
		char myIp[20];    //przyznany adres IP
		char simStatus[20]; //status karty sim
		char sim_imsi[20]; //numer IMSI karty SIM
		RegStatus mRegStatus;
	} bgParam;

	struct {
		struct {
			uint8_t simCardInserted;
			bool simCardMsgEnabled;
			bool simPinOk;
			bool isCReg;
			bool iAct; //aktywne połączenie IP
		} rdy;

		struct {
			bool configured;
			bool recived;
			uint32_t tryTick;
			int tryCnt;
			uint32_t reciveTick;
			int recErrorCode;
			TDATE reciveTime;
		} ntpTime;

		struct {
			bool recived;
			bool isDayLightSaving;
			int quatersDiff;
			TDATE reciveTime;
		} gsmTime;

		struct {
			uint32_t recivedTick;
			uint32_t sndReqTick;
			bool recivedOK;
			int tryCnt;
			float rssi;
			float ber;
		} rssi;

		struct {
			uint32_t tryTick;
			int tryCnt;
			uint32_t reciveTick;
			bool gpsFix;
			int errorCode;
			float latitude;
			float longitude;
			float hdop; //  horizontal precision
			float altitude; // wysokość nap poz.morza
			int fix; // posiotion code
			float cog; // kierunek ruchu - kąt
			float spkm; // prędkość w km/h
			int satCnt;
			TDATE reciveTime;
		} gps;

		struct {
			bool svrOpened;
			int closeCode;
			int mSendMsgID;
			int errorSendCnt;
			int errorOpenCnt;
			uint32_t sentTick;
			uint32_t sentAckTick;
			uint32_t trySendTick;
			uint32_t tryOpenTick;
		} mqtt;
		struct {
			char nrTel[20];
			char msg[200];
			uint32_t reciveTick;
		} sms;

		struct {
			volatile bool flagSend;
			volatile bool flagSendText;
			char nrTel[20];
			char msg[200];
			uint32_t reciveTick;
		} sendSms;

		struct {
			int errorCnt;
			int uartRestartCnt;
		}uart;

	} state;

	Bg96Driver();
	void shell(OutStream *strem, const char *cmd);
public:
	BgPhase getPhase() {
		return mPhase;
	}
	bool isMqttSendingOk();
	bool isMqttAutoRun();

	bool isSimCardInserted(){
		return state.rdy.simCardInserted;
	}
	bool isNetworkRegistered(){
		return state.rdy.isCReg;
	}
	bool isIPready(){
		return state.rdy.iAct;
	}
	bool isMqttSvrOpened(){
		return state.mqtt.svrOpened;
	}

};

#endif /* BG96DRIVER_H_ */
