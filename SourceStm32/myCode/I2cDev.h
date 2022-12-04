/*
 * I2cDev.h
 *
 *  Created on: 30 gru 2020
 *      Author: Grzegorz
 */

#ifndef I2CDEV_H_
#define I2CDEV_H_

#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include "MsgStream.h"

class I2c1Dev {
	friend class I2c1Bus;
protected:
	uint8_t mDevAdr;
	bool mDevExist;
	void showDevExist(MsgStream *strm);
	virtual void tick() {
	}
	virtual void init() {
	}
public:
	virtual void showState(MsgStream *strm)=0;
	virtual void showMeas(MsgStream *strm) {
	}
	virtual void execFun(MsgStream *strm, int funNr) {
	}
	uint8_t getAdr() {
		return mDevAdr;
	}
	bool isDevExist() {
		return mDevExist;
	}
	virtual bool isError()=0;
};

class SHT35DevPub: public I2c1Dev {
public:
	uint32_t serialNr;
	HAL_StatusTypeDef mMeasStart;
	virtual HAL_StatusTypeDef getData(float *temperature, float *humidity)=0;
	static SHT35DevPub* createDev(uint8_t devAdr);
};

class Bmp338DevPub: public I2c1Dev {
public:
	bool chipIdOk;
	bool coefOk;
	virtual HAL_StatusTypeDef getData(float *temperature, float *pressure)=0;
	static Bmp338DevPub* createDev(uint8_t devAdr);
};

class I2c1Bus {
	friend class SHT35Dev;
	friend class Bmp338Dev;
	friend class I2c1Dev;
	friend class SSD1306Dev;
protected:
	enum {
		MAX_DEV_CNT = 4,
	};
	static I2C_HandleTypeDef hi2c;
	static osMutexId mBusMutex;
	static int mDevCnt;
	static I2c1Dev *devTab[MAX_DEV_CNT];
	static int mBusRestartCnt;

	static bool openMutex(int who, int tm);
	static void closeMutex();

	void swap(uint16_t *p);
	static uint16_t swapD(uint16_t d);
	void putSwap(uint8_t *p, uint16_t d);

	static HAL_StatusTypeDef checkDev(uint8_t dev_addr);
	static HAL_StatusTypeDef checkDevMtx(uint8_t dev_addr);

	static HAL_StatusTypeDef readByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data);
	static HAL_StatusTypeDef readWord(uint8_t devAddr, uint8_t regAddr, uint16_t *data);
	static HAL_StatusTypeDef readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data);

	static HAL_StatusTypeDef writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data);
	static HAL_StatusTypeDef writeWord(uint8_t devAddr, uint8_t regAddr, uint16_t data);
	static HAL_StatusTypeDef writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, const uint8_t *data);
private:
	static int mLastMutexWho;
	static int mMutexWho;
	static void ScanBus(MsgStream *strm);
	static void showState(MsgStream *strm);
	static void showMeas(MsgStream *strm);
	static void ShowBusRegisters(MsgStream *strm);

	static void execFun(MsgStream *strm, int idx);
	static HAL_StatusTypeDef _InitHd();
	static HAL_StatusTypeDef InitHd();
	static void setAsGpio();
	static void setGpioSDA(GPIO_PinState PinState);
	static void setGpioSCL(GPIO_PinState PinState);
	static bool getGpioSDA();
	static bool getGpioSCL();
	static void gpioSCLWave();
	static bool rdBusyFlag();

public:
	static HAL_StatusTypeDef BusInit();
	static HAL_StatusTypeDef BusRestart();
	static HAL_StatusTypeDef BusUnlock();

	static void shell(MsgStream *strm, const char *cmd);
	static void addDev(I2c1Dev *dev);
	static void tick();
	static int getBusRestartCnt() {
		return mBusRestartCnt;
	}
	static bool isError();
};

#endif /* I2CDEV_H_ */
