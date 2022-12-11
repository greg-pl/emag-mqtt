#ifndef __UART_BOARD__
#define __UART_BOARD__

#include "stm32f4xx_hal.h"
#include "stdint.h"

//------------------------------------------------------------------------------------------------------------
typedef struct{
	const char *dt;
	int len;
}Portion;

class RxTxBuf {
private:
	int mHead;
	int mTail;
	int mSize;
	int getFree();
public:
	char *mBuf;
	RxTxBuf(int size);
	void clear();
	bool addBuf(Portion *portion);
	bool add(char ch);
	bool getLinearPart(const char **pptr, int *cnt);
	bool readLn(char *buf, int max);
};



//------------------------------------------------------------------------------------------------------------


class TUart  {
public:
	enum {
	    myUART1=0,
	    myUART2,
	    myUART3,
	    myUART4,
	    myUART5,
		PORT_CNT = 5, //
	};
	enum{
		parityNONE=0,
		parityEVEN, //Parzysty
		parityODD,  //Nieparzysty
	};
private:
	volatile bool txSending;
	int mPriority;

private:
	static TUart *UartVec[PORT_CNT];

protected:
	int mPortNr;  // 0..PORT_CNT-1;
	int mFramingErrCnt;
	int mIrqCnt;
	UART_HandleTypeDef mHuart;

	//konfiguracja
	bool mUseRts;
	bool mHdwCtrl;
	bool mHalfDuplex;

	virtual void Tick1ms() {

	}
	virtual void TxCpltCallback();


	virtual void RxCpltCallback(){

	}
	virtual void ErrorCallback(){

	}

	bool isSending(){
		//return (mHuart.RxState!=HAL_UART_STATE_READY);
		return txSending;
	}

public:
	static void TxCpltCallback(UART_HandleTypeDef *huart);
	static void RxCpltCallback(UART_HandleTypeDef *huart);
	static void ErrorCallback(UART_HandleTypeDef *huart);
	static void ISR(int uartNr);
	static void Tick1msEntry();
public:
	TUart(int PortNr, int Priority);
	virtual ~TUart(void);
	void uartAbort();
	bool isRxOverrun();

	HAL_StatusTypeDef Init(int BaudRate, int parity);
	HAL_StatusTypeDef Init(int BaudRate);
	void writeBuf(const void *dt, int len);
};

extern "C" void uartsTick1ms();

#endif
