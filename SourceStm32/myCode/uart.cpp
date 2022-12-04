#include "_ansi.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include "stdarg.h"

#include "main.h"
#include "uart.h"
#include "stm32f4xx_it.h"

//-----------------------------------------------------------------------------
// TUart
//-----------------------------------------------------------------------------

RxTxBuf::RxTxBuf(int size) {
	mSize = size;
	mBuf = (char*) malloc(mSize);
	clear();
}

void RxTxBuf::clear() {
	mHead = 0;
	mTail = 0;
	memset(mBuf, 0, mSize);
}

int RxTxBuf::getFree() {
	int ocu = mHead - mTail;
	if (ocu < 0)
		ocu += mSize;
	return mSize - 1 - ocu;
}

bool RxTxBuf::add(char ch) {
	int h = mHead;

	if (++h >= mSize)
		h = 0;
	if (h == mTail)
		return false;
	mBuf[mHead] = ch;
	mHead = h;
	return true;
}

bool RxTxBuf::addBuf(Portion *portion) {
	int fr = getFree();
	if (fr == 0) {
		//nic nie udało sie odłożyć, porcja bez zmian
		return false;
	}

	int lenP = portion->len;
	if (lenP > fr)
		lenP = fr;

	int len = lenP;
	int n1 = mSize - mHead;
	int len1 = len;
	if (len1 > n1) {
		len1 = n1;
	}
	const char *dt = portion->dt;
	memcpy(&mBuf[mHead], dt, len1);
	mHead += len1;
	if (mHead >= mSize)
		mHead = 0;
	len -= len1;
	if (len != 0) {
		dt += len1;
		memcpy(&mBuf[mHead], dt, len);
		mHead += len;
	}
	portion->len -= lenP;
	(portion->dt) += lenP;

	return (portion->len == 0);
}

bool RxTxBuf::getLinearPart(const char **pptr, int *cnt) {
	if (mTail == mHead)
		return false;
	int hh = mHead;
	int tt = mTail;

	int n;
	int nTT;
	if (hh > tt) {
		n = hh - tt;
		nTT = hh;
	} else {
		n = mSize - tt;
		nTT = 0;
	}
	*pptr = &mBuf[tt];
	*cnt = n;
	mTail = nTT;
	return true;
}

bool RxTxBuf::readLn(char *buf, int max) {
	int t = mTail;
	int h = mHead;
	int k = 0;
	while (t != h) {
		char ch = mBuf[t];
		if (++t >= mSize)
			t = 0;

		if (ch == '\n') {
			mTail = t;
			buf[k++] = ch;
			buf[k++] = 0;
			return true;
		}
		if (k < max - 1) {
			buf[k++] = ch;
		}
	}
	return false;
}

//-----------------------------------------------------------------------------
// TUart
//-----------------------------------------------------------------------------

//PortNr: 1...N
const USART_TypeDef *const USART_TAB[TUart::PORT_CNT] = { USART1, USART2, USART3, UART4, UART5 };
const IRQn_Type IRQ_NR_TAB[TUart::PORT_CNT] = { USART1_IRQn, USART2_IRQn, USART3_IRQn, UART4_IRQn, UART5_IRQn };

TUart *TUart::UartVec[TUart::PORT_CNT] = { NULL, NULL, NULL, NULL, NULL };

TUart::TUart(int PortNr, int Priority) {

	mPortNr = PortNr;
	if (mPortNr >= 0 && mPortNr < PORT_CNT) {
		UartVec[PortNr] = this;
		memset(&mHuart, 0, sizeof(mHuart));

		mPriority = Priority;
		mFramingErrCnt = 0;
		mIrqCnt = 0;
		txSending = false;
		mUseRts = false;
		mHdwCtrl = false;
		mHalfDuplex = false;

	} else {
#ifdef  USE_FULL_ASSERT
		assert_failed((byte*) 0, __LINE__);
#endif
	}
}

TUart::~TUart(void) {
	UartVec[mPortNr] = NULL;
	return;
}

void TUart::uartAbort() {
	HAL_UART_Abort(&mHuart);
	txSending = false;
}


bool TUart::isRxOverrun(){
	uint32_t isrflags   = READ_REG(mHuart.Instance->SR);
	return ((isrflags & USART_SR_ORE)!=0);
}



void TUart::TxCpltCallback() {
	txSending = false;
}

void TUart::writeBuf(const void *dt, int len) {
	if (len > 0) {
		txSending = true;
		HAL_UART_Transmit_IT(&mHuart, (uint8_t*) dt, len);
	}
}

HAL_StatusTypeDef TUart::Init(int BaudRate) {
	return Init(BaudRate, parityNONE);
}

HAL_StatusTypeDef TUart::Init(int BaudRate, int parity) {

	mHuart.Instance = (USART_TypeDef*) USART_TAB[mPortNr];
	mHuart.Init.BaudRate = BaudRate;
	mHuart.Init.StopBits = UART_STOPBITS_1;
	switch (parity) {
	default:
	case parityNONE:
		mHuart.Init.Parity = UART_PARITY_NONE;
		mHuart.Init.WordLength = UART_WORDLENGTH_8B;
		break;
	case parityEVEN:
		mHuart.Init.Parity = UART_PARITY_EVEN;
		mHuart.Init.WordLength = UART_WORDLENGTH_9B;
		break;
	case parityODD:
		mHuart.Init.Parity = UART_PARITY_ODD;
		mHuart.Init.WordLength = UART_WORDLENGTH_9B;
		break;
	}
	mHuart.Init.Mode = UART_MODE_TX_RX;
	mHuart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	mHuart.Init.OverSampling = UART_OVERSAMPLING_16;

	mHuart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	if (mUseRts)
		mHuart.Init.HwFlowCtl = UART_HWCONTROL_RTS;
	if (mHdwCtrl)
		mHuart.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;

	HAL_StatusTypeDef st;
	if (!mHalfDuplex)
		st = HAL_UART_Init(&mHuart);
	else
		st = HAL_HalfDuplex_Init(&mHuart);

	// interrupt Init
	if (st == HAL_OK) {
		IRQn_Type irqNr = IRQ_NR_TAB[mPortNr];
		HAL_NVIC_SetPriority(irqNr, mPriority, 0);
		HAL_NVIC_EnableIRQ(irqNr);
	}
	return st;

//	TxQuee->Clear();
//	RxQuee->Clear();
//HAL_UART_Receive_IT(&mHuart, &rxRec.mRecByte, 1);
}

void TUart::Tick1msEntry() {
	for (int i = 0; i < PORT_CNT; i++) {
		if (UartVec[i] != NULL) {
			UartVec[i]->Tick1ms();
		}
	}
}

void TUart::TxCpltCallback(UART_HandleTypeDef *huart) {
	for (int i = 0; i < PORT_CNT; i++) {
		if (UartVec[i] != NULL) {
			if (&(UartVec[i]->mHuart) == huart) {
				UartVec[i]->TxCpltCallback();
				break;
			}
		}
	}
}

void TUart::RxCpltCallback(UART_HandleTypeDef *huart) {
	for (int i = 0; i < PORT_CNT; i++) {
		if (UartVec[i] != NULL) {
			if (&(UartVec[i]->mHuart) == huart) {
				UartVec[i]->RxCpltCallback();
				break;
			}
		}
	}
}

void TUart::ErrorCallback(UART_HandleTypeDef *huart) {
	for (int i = 0; i < PORT_CNT; i++) {
		if (UartVec[i] != NULL) {
			if (&(UartVec[i]->mHuart) == huart) {
				UartVec[i]->ErrorCallback();
				break;
			}
		}
	}
}

void TUart::ISR(int uartNr) {
	TUart *uart = UartVec[uartNr];
	if (uart != NULL) {
		uart->mIrqCnt++;
		HAL_UART_IRQHandler(&uart->mHuart);
	}
}

extern "C" void uartsTick1ms() {
	TUart::Tick1msEntry();
}

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	TUart::TxCpltCallback(huart);
}

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	TUart::RxCpltCallback(huart);
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	TUart::ErrorCallback(huart);
}

extern "C" void USART1_IRQHandler(void) {
	TUart::ISR(TUart::myUART1);
}
extern "C" void USART2_IRQHandler(void) {
	TUart::ISR(TUart::myUART2);
}
extern "C" void USART3_IRQHandler(void) {
	TUart::ISR(TUart::myUART3);
}
extern "C" void UART4_IRQHandler(void) {
	TUart::ISR(TUart::myUART4);
}
extern "C" void UART5_IRQHandler(void) {
	TUart::ISR(TUart::myUART5);
}

