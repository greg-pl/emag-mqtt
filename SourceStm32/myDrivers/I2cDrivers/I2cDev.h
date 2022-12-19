/*
 * I2cDev.h
 *
 *  Created on: 30 gru 2020
 *      Author: Grzegorz
 */

#ifndef I2CDEV_H_
#define I2CDEV_H_

#include <UniDev.h>
#include <IOStream.h>
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include "shellItem.h"

#define FILTR_FACTOR    0.8
#define TIME_DT_RD      2000
#define TIME_DT_VALID   5000
#define TIME_REINIT	    6000

class I2cBus;

class I2cDev: public UniDev {
	friend class I2cBus;

protected:
	I2cBus *mBus;
	uint8_t mDevAdr;
	bool mDevExist;
	void showDevExist(OutStream *strm);
	virtual void tick() {
	}
	virtual void init() {
	}
protected:
	bool openMutex(int who, int tm);
	void closeMutex();

	HAL_StatusTypeDef checkDev();
	HAL_StatusTypeDef checkDevMtx();
	HAL_StatusTypeDef Transmit(uint16_t len, uint8_t *data);

	HAL_StatusTypeDef readByte(uint8_t regAddr, uint8_t *data);
	HAL_StatusTypeDef readWord(uint8_t regAddr, uint16_t *data);
	HAL_StatusTypeDef readBytes(uint8_t regAddr, uint8_t length, uint8_t *data);
	HAL_StatusTypeDef readBytes16bit(uint16_t reg_addr, uint16_t len, uint8_t *data);

	HAL_StatusTypeDef writeByte(uint8_t regAddr, uint8_t data);
	HAL_StatusTypeDef writeWord(uint8_t regAddr, uint16_t data);
	HAL_StatusTypeDef writeBytes(uint8_t regAddr, uint8_t length, const uint8_t *data);
	HAL_StatusTypeDef writeBytes16bit(uint8_t reg_addr, uint16_t len, const uint8_t *data);

	virtual void shell(OutStream *strm, const char *cmd);

public:
	I2cDev(I2cBus *bus, uint8_t adr, const char *name);
	virtual void showState(OutStream *strm)=0;
	virtual void showMeas(OutStream *strm) {
	}
};

class I2cBus {
	friend class I2cDev;
private:
	enum {
		MAX_DEV_CNT = 4,
	};
	struct {
		int lastWho;
		int who;
		osMutexId mutex;
	} mMuRec;

	struct {
		int cnt;
		I2cDev *tab[MAX_DEV_CNT];
	} devs;

	struct {
		ShellItemFx *menuTab;
		void **argTab;
	} menuExp;

	I2C_TypeDef *mI2cDef;
	I2C_HandleTypeDef hi2c;
	int mBusRestartCnt;

	bool openMutex(int who, int tm);
	void closeMutex();

	void swap(uint16_t *p);
	static uint16_t swapD(uint16_t d);
	void putSwap(uint8_t *p, uint16_t d);
private:
	HAL_StatusTypeDef checkDev(uint8_t dev_addr);
	HAL_StatusTypeDef checkDevMtx(uint8_t dev_addr);
	HAL_StatusTypeDef Transmit(uint8_t dev_addr, uint16_t len, uint8_t *data);

	HAL_StatusTypeDef readByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data);
	HAL_StatusTypeDef readWord(uint8_t devAddr, uint8_t regAddr, uint16_t *data);
	HAL_StatusTypeDef readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data);
	HAL_StatusTypeDef readBytes16bit(uint8_t dev_addr, uint16_t reg_addr, uint16_t len, uint8_t *data);

	HAL_StatusTypeDef writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data);
	HAL_StatusTypeDef writeWord(uint8_t devAddr, uint8_t regAddr, uint16_t data);
	HAL_StatusTypeDef writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, const uint8_t *data);
	HAL_StatusTypeDef writeBytes16bit(uint8_t dev_addr, uint8_t reg_addr, uint16_t len, const uint8_t *data);

	void ScanBus(OutStream *strm);
	void showState(OutStream *strm);
	void showMeas(OutStream *strm);
	void ShowBusRegisters(OutStream *strm);

	HAL_StatusTypeDef _InitHd();
	HAL_StatusTypeDef InitHd();
	void setAsGpio();
	void setGpioSDA(GPIO_PinState PinState);
	void setGpioSCL(GPIO_PinState PinState);
	bool getGpioSDA();
	bool getGpioSCL();
	void gpioSCLWave();
	bool rdBusyFlag();

public:
	static void funShowState(OutStream *strm, const char *cmd, void *arg);
	static void funShowMeasure(OutStream *strm, const char *cmd, void *arg);
	static void funScan(OutStream *strm, const char *cmd, void *arg);
	static void funRestart(OutStream *strm, const char *cmd, void *arg);
	static void funShowReg(OutStream *strm, const char *cmd, void *arg);
	static void funUnblock(OutStream *strm, const char *cmd, void *arg);
	static void funRdGpio(OutStream *strm, const char *cmd, void *arg);
	static void funMakeWave(OutStream *strm, const char *cmd, void *arg);
	static void funForChild(OutStream *strm, const char *cmd, void *arg);

public:
	I2cBus(I2C_TypeDef *i2cDef);
	HAL_StatusTypeDef BusRestart();
	HAL_StatusTypeDef BusUnlock();

	void shell(OutStream *strm, const char *cmd);
	void addDev(I2cDev *dev);
	void tick();
	int getBusRestartCnt() {
		return mBusRestartCnt;
	}
	bool isDataError();
};

#endif /* I2CDEV_H_ */
