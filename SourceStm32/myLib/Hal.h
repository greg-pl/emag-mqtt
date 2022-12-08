/*
 * Hal.h
 *
 *  Created on: 8 gru 2022
 *      Author: Grzegorz
 */

#ifndef HAL_H_
#define HAL_H_

#include "utils.h"

class Hdw {
private:
	static ST3 getPinCfg(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
public:
	static void setPinAsInpNoPull(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
	static void setPinAsInpPullUp(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
	static void setPinAsInpPullDn(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
	static void setPinAsOutputPP(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
public:
	static void led3k(bool red, bool geen, bool blue);
	static void phyReset(bool activ);
	static void phyPower(bool poweOn);

	static void heaterOn(bool on);
	static bool getHeaterOn();
	static bool getHeaterFlg();

	static void dustSensorOn(bool on);
	static bool getDustSensorOn();
	static bool getDustSensorFlg();
	static void dustSleepOn(bool on);
	static bool getDustSleepOn();

	static ST3 getPinCfg0();
	static ST3 getPinCfg1();
	static ST3 getPinCfg2();
	static ST3 getPinCfg3();

};




#endif /* HAL_H_ */
