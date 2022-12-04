/*
 * shell.h
 *
 *  Created on: Dec 5, 2020
 *      Author: Grzegorz
 */

#ifndef SHELL_H_
#define SHELL_H_

#include "uart.h"
#include "TaskClass.h"
#include "EscTerminal.h"
#include "MsgStream.h"

//------------------------------------------------------------------------------------------------------------
// klasa virtualna
class ShellConnection {
protected:
	enum {
		TX_BUF_SIZE = 2048,  //bufor do nadawania
		RX_BUF_SIZE = 64,  //
	};
	osThreadId mThreadId;
	RxTxBuf *txBuf;
	struct {
		char buf[64];
		int head;
		int tail;
	} rxRec;

	virtual bool isConnSending() =0;
	virtual void sendBuf(const void *ptr, int len)=0;
	virtual bool isReady(){
		return true;
	}
	void startSendNextPart();
public:
	int mReStartCnt;
	ShellConnection();
	void setThreadId(osThreadId threadId) {
		mThreadId = threadId;
	}
	virtual HAL_StatusTypeDef Init()=0;
	bool writeData(Portion *portion);
	bool getChar(char *ch);
};

//------------------------------------------------------------------------------------------------------------

class UartConnection: public TUart, public ShellConnection {
	friend class ShellTask;
private:
	char rxChar;
	void StartRecive();
protected:
	//TUart
	virtual void TxCpltCallback();
	virtual void RxCpltCallback();
	virtual void ErrorCallback();

protected:
	//ShellConnection
	virtual bool isConnSending();
	virtual void sendBuf(const void *ptr, int len);
public:
	UartConnection();
	virtual HAL_StatusTypeDef Init();

};

//------------------------------------------------------------------------------------------------------------

class USBConnection: public ShellConnection {
	friend class ShellTask;
private:
	bool mReady;
	uint32_t mTransmitTick;

protected:
	//ShellConnection
	virtual bool isConnSending();
	virtual void sendBuf(const void *ptr, int len);
	virtual bool isReady();

public:
	static USBConnection *Me;
	void inpDataFun(uint8_t *Buf, uint32_t Len);
	void transmitCplt();
	void initCDC();
public:
	USBConnection();
	virtual HAL_StatusTypeDef Init();
};

//------------------------------------------------------------------------------------------------------------

class ShellTask: public TaskClass, public TermStream, public MsgStream {
public:
	enum {
		SIGNAL_CHAR = 0x01, //
		SIGNAL_MSG = 0x02, //
	};
private:
	static ShellTask *Me;
	ShellConnection *myConnection;
	EscTerminal *term;

	osMutexId mOutTxtMutex;
	char outBuf[200];  //dostęp do bufora tylko po otwarciu semafora
	bool flgSendAny;
	int mNoTermSmfCnt; // licznik gdy zajęty semator
	int mFullTxCnt;  // licznik gdy przepełniony bufor TX

	void execCmdLine(const char *cmd);
	void execAltChar(char altChar);
	void execFunKey(FunKey funKey);
	const char* getColorStr(TermColor color);

	void ethMenu(const char *cmd);
	void timeMenu(const char *cmd);
	void netMenu(const char *cmd);

	void showHdwState();
	void showThreadList();
	void showMemInfo();
	void showDevState();
	int Ping(ip4_addr_t addr, int length);

protected:
	//TermStream
	virtual void putOut(const void *mem, int len);
	virtual bool openOutMutex(int tm);
	virtual void closeOutMutex();
	void putOutStr(const char *str);

protected:
	//TaskClass
	virtual void ThreadFunc();
private:
public:
	//MsgStream
	virtual void msgAp(TermColor color, const char *pFormat, va_list ap);
	virtual void msg(TermColor color, const char *pFormat, ...);
	virtual bool msgOpen(TermColor color);
	virtual void msgClose();
	virtual void msgItemWr(const char *txt);
	virtual void msgItem(const char *pFormat, ...);
	virtual void dumpBuf(TermColor color, const char *buf);
public:
	ShellTask();
	static void _Msg(TermColor color, const char *pFormat, ...);

};

#endif /* SHELL_H_ */