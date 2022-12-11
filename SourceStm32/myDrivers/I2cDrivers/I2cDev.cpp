/*
 * I2cDev.cpp
 *
 *  Created on: 30 gru 2020
 *      Author: Grzegorz
 */

#include "string.h"
#include "stdio.h"
#include "math.h"

#include <I2cDev.h>
#include <main.h>
#include <Utils.h>
#include <cpx.h>
#include "Shell.h"
#include <ShellItem.h>

extern ShellTask *shellTask;

I2cBus::I2cBus(I2C_TypeDef *i2cDef) {
	mI2cDef = i2cDef;
	menuExp.menuTab = NULL;
	menuExp.argTab = NULL;

	devs.cnt = 0;
	for (int i = 0; i < MAX_DEV_CNT; i++) {
		devs.tab[i] = NULL;
	}
	mBusRestartCnt = 0;

	osMutexDef(I2C1Dev);
	mMuRec.mutex = osMutexCreate(osMutex(I2C1Dev));

	InitHd();
}

HAL_StatusTypeDef I2cBus::_InitHd() {
	memset(&hi2c, 0, sizeof(hi2c));
	hi2c.Instance = mI2cDef;
	hi2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c.Init.ClockSpeed = 100000; //100kHz
	hi2c.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c.Init.OwnAddress1 = 0;
	hi2c.Init.OwnAddress2 = 0;
	hi2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	return HAL_I2C_Init(&hi2c);
}

HAL_StatusTypeDef I2cBus::InitHd() {

	HAL_StatusTypeDef st = HAL_BUSY;
	if (openMutex(1, 100)) {
		st = _InitHd();
		closeMutex();
	}
	return st;
}

HAL_StatusTypeDef I2cBus::BusUnlock() {
	if (openMutex(1, 100)) {
		SET_BIT(hi2c.Instance->CR1, I2C_CR1_SWRST);
		HAL_Delay(2);
		CLEAR_BIT(hi2c.Instance->CR1, I2C_CR1_SWRST);
		closeMutex();
	}
	return HAL_OK;
}

//PB6     ------> I2C1_SCL
//PB7     ------> I2C1_SDA

void I2cBus::setAsGpio() {
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	setGpioSDA(GPIO_PIN_SET);
	setGpioSCL(GPIO_PIN_SET);
}

void I2cBus::setGpioSDA(GPIO_PinState PinState) {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, PinState);
}
void I2cBus::setGpioSCL(GPIO_PinState PinState) {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, PinState);
}

bool I2cBus::getGpioSDA() {
	return (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET);
}

bool I2cBus::getGpioSCL() {
	return (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_SET);
}

void I2cBus::gpioSCLWave() {
	setAsGpio();
	for (int i = 0; i < 64; i++) {
		setGpioSCL(GPIO_PIN_RESET);
		osDelay(2);
		setGpioSCL(GPIO_PIN_SET);
		osDelay(2);
	}
}

HAL_StatusTypeDef I2cBus::BusRestart() {
	HAL_I2C_DeInit(&hi2c);
	HAL_StatusTypeDef st = InitHd();
	if (st == HAL_OK) {
		for (int i = 0; i < devs.cnt; i++) {
			devs.tab[i]->init();
		}
	}
	return st;
}

void I2cBus::ShowBusRegisters(OutStream *strm) {
	if (strm->oOpen(colWHITE)) {
		strm->oMsg("CR1:0x%08X", hi2c.Instance->CR1);
		strm->oMsg("CR2:0x%08X", hi2c.Instance->CR2);
		strm->oMsg("SR1:0x%08X", hi2c.Instance->SR1);
		strm->oMsg("SR2:0x%08X", hi2c.Instance->SR2);
		strm->oMsg("CCR  :0x%08X", hi2c.Instance->CCR);
		strm->oMsg("TRISE:0x%08X", hi2c.Instance->TRISE);
		strm->oMsg("FLTR :0x%08X", hi2c.Instance->FLTR);

		strm->oClose();
	}

}

bool I2cBus::openMutex(int who, int tm) {
	bool q = (osMutexWait(mMuRec.mutex, tm) == osOK);
	if (q)
		mMuRec.who = who;
	return q;
}
void I2cBus::closeMutex() {
	mMuRec.lastWho = mMuRec.who;
	mMuRec.who = 0;
	osMutexRelease(mMuRec.mutex);
}

void I2cBus::swap(uint16_t *p) {
	uint8_t *pb = (uint8_t*) p;
	uint8_t b1 = pb[0];
	uint8_t b2 = pb[1];
	pb[0] = b2;
	pb[1] = b1;
}

uint16_t I2cBus::swapD(uint16_t d) {
	return (d >> 8) | (d << 8);
}

void I2cBus::putSwap(uint8_t *p, uint16_t d) {
	p[0] = d >> 8;
	p[1] = d & 0xff;
}

HAL_StatusTypeDef I2cBus::checkDev(uint8_t dev_addr) {
	return HAL_I2C_IsDeviceReady(&hi2c, dev_addr, 3, 100);
}

HAL_StatusTypeDef I2cBus::Transmit(uint8_t dev_addr, uint16_t len, uint8_t *data) {
	return HAL_I2C_Master_Transmit(&hi2c, dev_addr, data, len, 100);
}

HAL_StatusTypeDef I2cBus::readBytes16bit(uint8_t dev_addr, uint16_t reg_addr, uint16_t len, uint8_t *data) {
	return HAL_I2C_Mem_Read(&hi2c, dev_addr, reg_addr, I2C_MEMADD_SIZE_16BIT, data, len, 100);
}

HAL_StatusTypeDef I2cBus::readBytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t len, uint8_t *data) {
	return HAL_I2C_Mem_Read(&hi2c, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, 100);
}

HAL_StatusTypeDef I2cBus::readByte(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data) {
	return readBytes(dev_addr, reg_addr, 1, data);
}

HAL_StatusTypeDef I2cBus::readWord(uint8_t dev_addr, uint8_t reg_addr, uint16_t *data) {
	uint16_t tmp;
	HAL_StatusTypeDef st = readBytes(dev_addr, reg_addr, 2, (uint8_t*) &tmp);
	*data = swapD(tmp);
	return st;
}

HAL_StatusTypeDef I2cBus::writeBytes16bit(uint8_t dev_addr, uint8_t reg_addr, uint16_t len, const uint8_t *data) {
	return HAL_I2C_Mem_Write(&hi2c, dev_addr, reg_addr, I2C_MEMADD_SIZE_16BIT, (uint8_t*) (int) data, len, 100);
}
HAL_StatusTypeDef I2cBus::writeBytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t len, const uint8_t *data) {
	return HAL_I2C_Mem_Write(&hi2c, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, (uint8_t*) (int) data, len, 100);
}

HAL_StatusTypeDef I2cBus::writeByte(uint8_t dev_addr, uint8_t reg_addr, uint8_t data) {
	return writeBytes(dev_addr, reg_addr, 1, &data);
}

HAL_StatusTypeDef I2cBus::writeWord(uint8_t dev_addr, uint8_t reg_addr, uint16_t data) {
	data = swapD(data);
	return writeBytes(dev_addr, reg_addr, 2, (const uint8_t*) &data);
}

void I2cBus::addDev(I2cDev *dev) {
	if (devs.cnt < MAX_DEV_CNT) {
		devs.tab[devs.cnt] = dev;
		devs.cnt++;
	}
}

HAL_StatusTypeDef I2cBus::checkDevMtx(uint8_t dev_addr) {
	HAL_StatusTypeDef st = HAL_NO_SEMF;
	if (I2cBus::openMutex(2, 100)) {
		st = checkDev(dev_addr);
		I2cBus::closeMutex();
	}
	return st;
}

void I2cBus::ScanBus(OutStream *strm) {
	if (strm->oOpen(colWHITE)) {
		if (openMutex(3, 100)) {
			int devCnt = 0;
			for (int i = 0; i < 128; i++) {
				uint16_t dev_addr = 2 * i;
				if (HAL_I2C_IsDeviceReady(&hi2c, dev_addr, 2, 50) == HAL_OK) {
					strm->oMsg("Found adr=0x%02X", dev_addr);
					devCnt++;
				}
			}
			strm->oMsg("Found %u devices", devCnt);
			closeMutex();
		} else {
			strm->oMsg("I2c mutex error");
		}
		strm->oClose();
	}
}

void I2cBus::showState(OutStream *strm) {
	if (strm->oOpen(colWHITE)) {
		strm->oMsg("MutextWho=%d (Last=%d)", mMuRec.who, mMuRec.lastWho);
		strm->oMsg("BusRestartCnt=%u", mBusRestartCnt);

		for (int i = 0; i < devs.cnt; i++) {
			devs.tab[i]->showState(strm);
		}
		strm->oClose();
	}
}
void I2cBus::showMeas(OutStream *strm) {
	if (strm->oOpen(colWHITE)) {
		for (int i = 0; i < devs.cnt; i++) {
			devs.tab[i]->showMeas(strm);
		}
		strm->oClose();
	}
}

bool I2cBus::rdBusyFlag() {
	return ((hi2c.Instance->SR2 & I2C_SR2_BUSY) != 0);
}
void I2cBus::tick() {
	if (rdBusyFlag()) {
		//zablokowanie magistrali- próba odblokowania
		if (openMutex(4, 100)) {
			setAsGpio();
			bool sdaBf = getGpioSDA();
			gpioSCLWave();
			bool sdaAf = getGpioSDA();
			bool busyAf = rdBusyFlag();
			closeMutex();
			BusRestart();
			shellTask->oMsgX(colRED, "I2CBus RESTART: sdaBf=%u, sdaAf=%u, busyAf=%u", sdaBf, sdaAf, busyAf);
			mBusRestartCnt++;
		}

	} else {
		for (int i = 0; i < devs.cnt; i++) {
			devs.tab[i]->tick();
		}
	}
}

bool I2cBus::isError() {
	bool q = 0;
	for (int i = 0; i < devs.cnt; i++) {
		q |= devs.tab[i]->isError();
	}
	return q;
}

void I2cBus::funShowState(OutStream *strm, const char *cmd, void *arg) {
	I2cBus *dev = (I2cBus*) arg;
	dev->showState(strm);
}

void I2cBus::funShowMeasure(OutStream *strm, const char *cmd, void *arg) {
	I2cBus *dev = (I2cBus*) arg;
	dev->showMeas(strm);
}
void I2cBus::funScan(OutStream *strm, const char *cmd, void *arg) {
	I2cBus *dev = (I2cBus*) arg;
	dev->ScanBus(strm);
}

void I2cBus::funRestart(OutStream *strm, const char *cmd, void *arg) {
	I2cBus *dev = (I2cBus*) arg;
	dev->BusRestart();
}

void I2cBus::funShowReg(OutStream *strm, const char *cmd, void *arg) {
	I2cBus *dev = (I2cBus*) arg;
	dev->ShowBusRegisters(strm);
}

void I2cBus::funUnblock(OutStream *strm, const char *cmd, void *arg) {
	I2cBus *dev = (I2cBus*) arg;
	dev->BusUnlock();
	strm->oMsgX(colWHITE, "Ok. Wykonaj restart i2c.");
}

void I2cBus::funRdGpio(OutStream *strm, const char *cmd, void *arg) {
	I2cBus *dev = (I2cBus*) arg;
	if (strm->oOpen(colWHITE)) {
		dev->setAsGpio();
		strm->oMsg("SDA:%u", dev->getGpioSDA());
		strm->oMsg("SCL:%u", dev->getGpioSCL());
		strm->oClose();
	}
}
void I2cBus::funMakeWave(OutStream *strm, const char *cmd, void *arg) {
	I2cBus *dev = (I2cBus*) arg;
	dev->gpioSCLWave();
	strm->oMsgX(colWHITE, "SCLWave. Wykonaj restart i2c.");
}

void I2cBus::funForChild(OutStream *strm, const char *cmd, void *arg) {
	I2cDev *dev = (I2cDev*) arg;
	dev->shell(strm, cmd);
}

const ShellItemFx menuI2CFx[] = { //
		{ "s", "stan", I2cBus::funShowState }, //
				{ "m", "pomiary", I2cBus::funShowMeasure }, //
				{ "scan", "przeszukanie magistrali", I2cBus::funScan }, //
				{ "restart", "restart magistrali", I2cBus::funRestart }, //
				{ "reg", "show iic registers", I2cBus::funShowReg }, //
				{ "busUnlock", "odblokowanie magistrali i2c", I2cBus::funUnblock }, //
				{ "rdGpio", "czytaj linie SDA,SCL jako GPIO", I2cBus::funRdGpio }, //
				{ "sclWave", "wygenerowanie fali na lnii SCL", I2cBus::funMakeWave }, //
				{ NULL, NULL } };

void I2cBus::shell(OutStream *strm, const char *cmd) {

	if (menuExp.menuTab == NULL) {
		int cnt = sizeof(menuI2CFx) / sizeof(ShellItemFx);
		cnt += devs.cnt;
		menuExp.argTab = (void**) malloc((cnt + devs.cnt) * sizeof(void*));
		menuExp.menuTab = (ShellItemFx*) malloc((cnt + devs.cnt) * sizeof(ShellItemFx));

		if (menuExp.menuTab != NULL) {
			memcpy(&menuExp.menuTab[devs.cnt], menuI2CFx, cnt * sizeof(ShellItemFx));
			for (int i = 0; i < cnt; i++) {
				menuExp.argTab[devs.cnt + i] = this;
			}

			for (int i = 0; i < devs.cnt; i++) {
				menuExp.menuTab[i].cmd = devs.tab[i]->mName;
				menuExp.menuTab[i].descr = ">>>";
				menuExp.menuTab[i].fun = funForChild;
				menuExp.argTab[i] = devs.tab[i];
			}
		}
	}
	if (menuExp.menuTab != NULL)
		execMenuCmdArg(strm, menuExp.menuTab, cmd, menuExp.argTab, "I2C Menu");
}

//-------------------------------------------------------------------------------------------------------------------------
// I2c1Dev
//-------------------------------------------------------------------------------------------------------------------------
I2cDev::I2cDev(I2cBus *bus, uint8_t adr, const char *name) :
		UniDev::UniDev(name) {
	mBus = bus;
	mDevAdr = adr;
	mDevExist = false;
	strlcpy(mName, name, sizeof(mName));
	mBus->addDev(this);
}

void I2cDev::shell(OutStream *strm, const char *cmd) {

}

void I2cDev::showDevExist(OutStream *strm) {
	HAL_StatusTypeDef st = mBus->checkDevMtx(mDevAdr);
	strm->oMsg("%s DevExist=%s", mName, HAL_getErrStr(st));
}

HAL_StatusTypeDef I2cDev::checkDev() {
	return mBus->checkDev(mDevAdr);
}
HAL_StatusTypeDef I2cDev::checkDevMtx() {
	return mBus->checkDevMtx(mDevAdr);
}
HAL_StatusTypeDef I2cDev::Transmit(uint16_t len, uint8_t *data) {
	return mBus->Transmit(mDevAdr, len, data);
}

HAL_StatusTypeDef I2cDev::readByte(uint8_t regAddr, uint8_t *data) {
	return mBus->readByte(mDevAdr, regAddr, data);
}
HAL_StatusTypeDef I2cDev::readWord(uint8_t regAddr, uint16_t *data) {
	return mBus->readWord(mDevAdr, regAddr, data);
}

HAL_StatusTypeDef I2cDev::readBytes(uint8_t regAddr, uint8_t length, uint8_t *data) {
	return mBus->readBytes(mDevAdr, regAddr, length, data);
}

HAL_StatusTypeDef I2cDev::readBytes16bit(uint16_t reg_addr, uint16_t len, uint8_t *data) {
	return mBus->readBytes16bit(mDevAdr, reg_addr, len, data);
}

HAL_StatusTypeDef I2cDev::writeByte(uint8_t regAddr, uint8_t data) {
	return mBus->writeByte(mDevAdr, regAddr, data);
}
HAL_StatusTypeDef I2cDev::writeWord(uint8_t regAddr, uint16_t data) {
	return mBus->writeWord(mDevAdr, regAddr, data);
}
HAL_StatusTypeDef I2cDev::writeBytes(uint8_t regAddr, uint8_t length, const uint8_t *data) {
	return mBus->writeBytes(mDevAdr, regAddr, length, data);
}
HAL_StatusTypeDef I2cDev::writeBytes16bit(uint8_t reg_addr, uint16_t len, const uint8_t *data) {
	return mBus->writeBytes16bit(mDevAdr, reg_addr, len, data);
}

bool I2cDev::openMutex(int who, int tm) {
	return mBus->openMutex(who, tm);
}

void I2cDev::closeMutex() {
	mBus->closeMutex();
}

