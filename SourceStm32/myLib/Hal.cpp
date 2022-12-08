/*
 * Hal.cpp
 *
 *  Created on: 8 gru 2022
 *      Author: Grzegorz
 */

#include <Hal.h>

//-------------------------------------------------------------------------------------
// Hdw
//-------------------------------------------------------------------------------------

void Hdw::setPinAsInpNoPull(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
	GPIO_InitTypeDef R = { 0 };
	R.Pin = GPIO_Pin;
	R.Mode = GPIO_MODE_INPUT;
	R.Pull = GPIO_NOPULL;
	R.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &R);
}

void Hdw::setPinAsInpPullUp(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
	GPIO_InitTypeDef R = { 0 };
	R.Pin = GPIO_Pin;
	R.Mode = GPIO_MODE_INPUT;
	R.Pull = GPIO_PULLUP;
	R.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &R);
}

void Hdw::setPinAsInpPullDn(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
	GPIO_InitTypeDef R = { 0 };
	R.Pin = GPIO_Pin;
	R.Mode = GPIO_MODE_INPUT;
	R.Pull = GPIO_PULLDOWN;
	R.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &R);
}

void Hdw::setPinAsOutputPP(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
	GPIO_InitTypeDef R = { 0 };
	R.Pin = GPIO_Pin;
	R.Mode = GPIO_MODE_OUTPUT_PP;
	R.Pull = GPIO_NOPULL;
	R.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &R);
}

ST3 Hdw::getPinCfg(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
	setPinAsInpPullUp(GPIOx, GPIO_Pin);
	bool qu = HAL_GPIO_ReadPin(GPIOx, GPIO_Pin);
	setPinAsInpPullDn(GPIOx, GPIO_Pin);
	bool qd = HAL_GPIO_ReadPin(GPIOx, GPIO_Pin);
	setPinAsInpNoPull(GPIOx, GPIO_Pin);
	if (qu & qd)
		return posVCC;
	if (!qu & !qd)
		return posGND;
	return posFREE;

}

ST3 Hdw::getPinCfg0() {
	return getPinCfg(CFG0_GPIO_Port, CFG0_Pin);
}
ST3 Hdw::getPinCfg1() {
	return getPinCfg(CFG1_GPIO_Port, CFG1_Pin);
}
ST3 Hdw::getPinCfg2() {
	return getPinCfg(CFG2_GPIO_Port, CFG2_Pin);
}
ST3 Hdw::getPinCfg3() {
	return getPinCfg(CFG3_GPIO_Port, CFG3_Pin);
}

void Hdw::led3k(bool red, bool green, bool blue) {

	if (red)
		HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);

	if (green)
		HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);

	if (blue)
		HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
}

void Hdw::heaterOn(bool on) {
	if (on)
		HAL_GPIO_WritePin(HEAT_PWR_EN_GPIO_Port, HEAT_PWR_EN_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(HEAT_PWR_EN_GPIO_Port, HEAT_PWR_EN_Pin, GPIO_PIN_SET);
}
bool Hdw::getHeaterOn() {
	return (HAL_GPIO_ReadPin(HEAT_PWR_EN_GPIO_Port, HEAT_PWR_EN_Pin) == GPIO_PIN_RESET);
}

//true -> zwarcie albo przegrzanie
bool Hdw::getHeaterFlg() {
	return (HAL_GPIO_ReadPin(HEAT_FLG_GPIO_Port, HEAT_FLG_Pin) == GPIO_PIN_RESET);
}

void Hdw::dustSensorOn(bool on) {
	setPinAsOutputPP(PYL_PWR_EN_GPIO_Port, PYL_PWR_EN_Pin);
	if (on)
		HAL_GPIO_WritePin(PYL_PWR_EN_GPIO_Port, PYL_PWR_EN_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(PYL_PWR_EN_GPIO_Port, PYL_PWR_EN_Pin, GPIO_PIN_SET);
}

bool Hdw::getDustSensorOn() {
	return (HAL_GPIO_ReadPin(PYL_PWR_EN_GPIO_Port, PYL_PWR_EN_Pin) == GPIO_PIN_RESET);
}

//true -> zwarcie albo przegrzanie
bool Hdw::getDustSensorFlg() {
	return (HAL_GPIO_ReadPin(PYL_FLG_GPIO_Port, PYL_FLG_Pin) == GPIO_PIN_RESET);

}

void Hdw::dustSleepOn(bool on) {
	if (on)
		HAL_GPIO_WritePin(PMS_SET_GPIO_Port, PMS_SET_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(PMS_SET_GPIO_Port, PMS_SET_Pin, GPIO_PIN_SET);
}
bool Hdw::getDustSleepOn() {
	return (HAL_GPIO_ReadPin(PMS_SET_GPIO_Port, PMS_SET_Pin) == GPIO_PIN_RESET);
}

void Hdw::phyReset(bool activ) {
	if (activ)
		HAL_GPIO_WritePin(ETH_PHY_RST_GPIO_Port, ETH_PHY_RST_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(ETH_PHY_RST_GPIO_Port, ETH_PHY_RST_Pin, GPIO_PIN_SET);
}

void Hdw::phyPower(bool poweOn) {
	if (poweOn)
		HAL_GPIO_WritePin(ETH_PWR_EN_GPIO_Port, ETH_PWR_EN_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(ETH_PWR_EN_GPIO_Port, ETH_PWR_EN_Pin, GPIO_PIN_SET);
}

