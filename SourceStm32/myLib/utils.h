/*
 * utils.h
 *
 *  Created on: Dec 5, 2020
 *      Author: Grzegorz
 */

#ifndef UTILS_H_
#define UTILS_H_

#include <IOStream.h>
#include "_ansi.h"
#include "stdint.h"
#include "stm32f4xx_hal.h"

#include "myDef.h"

typedef enum {
	posFREE = 0, posGND, posVCC,
} ST3;

extern "C" const char* ST3Str(ST3 val);
extern "C" const char* YN(bool q);
extern "C" const char* HL(bool q);
extern "C" const char* OnOff(bool q);
extern "C" const char* ErrOk(bool q);
extern "C" const char* OkErr(bool q);
extern "C" const char* FalseTrue(bool q);

extern "C" const char* HAL_getErrStr(HAL_StatusTypeDef st);
extern "C" TermColor HAL_getColor(HAL_StatusTypeDef st);
extern "C" const char* getTmSrcName(uint8_t tmSrc);

extern "C" bool strbcmp(const char *buf, const char *wz);
extern "C" bool strbcmp2(const char *buf, const char *wz, const char **rest);
extern "C" bool loadSoftVer(VerInfo *ver, const char *mem);

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

class Token {
private:
	static bool isSep(const char *sep, char ch);
public:
	static void trim(const char **inp);
	static bool get(const char **inp, const char *sepStr, char *tok, int size, char *pSep);
	static bool get(const char **inp, char *tok, int size, char *pSep);
	static bool get(const char **inp, char *tok, int size);
	static bool get(const char **inp);

	static bool getAsInt(const char **inp, int *val);
	static bool getAsInt(const char **inp, const char *sepStr, int *val);
	static bool getAsBool(const char **inp, bool *val);

	static bool getAsFloat(const char **inp, float *val);
	static bool chgCtrlChar(char *dst, const char *src, int max);
	static void remooveEOL(char *line);
	static bool remooveQuota(char *line);
	static void copyNoQuota(char *dst, int max, const char *src);
	static void setQuotaStr(char *dst, const char *txt, int max);
	static void shiftLeft(char *buf, int dist);
	static int parseInt(const char *ptr, int len);
};

class TimeTools {
public:
	static bool CheckTime(const TDATE *tm);
	static bool CheckDate(const TDATE *tm);
	static bool CheckDtTm(const TDATE *tm);
	static const char* TimeStr(char *buf, const TDATE *tm);
	static const char* TimeStrZZ(char *buf, const TDATE *tm);
	static const char* DateStr(char *buf, const TDATE *tm);
	static const char* DtTmStr(char *buf, const TDATE *tm);
	static const char* DtTmStrZZ(char *buf, const TDATE *tm);
	static bool parseTime(const char **cmd, TDATE *Tm);
	static bool parseDate(const char **cmd, TDATE *Tm);
	static void copyDate(TDATE *dst, const TDATE *src);
	static void copyTime(TDATE *dst, const TDATE *src);
	static bool AddHour(TDATE *tm, int delHour);
	static const char* TimeLongStr(char *buf, int milisec);
};

class Rtc {
private:
	static RTC_HandleTypeDef hrtc;
	static uint8_t getSetne(RTC_TimeTypeDef *sTime);
public:
	static HAL_StatusTypeDef mRtcStatus;
	static void Init();
	static bool ReadTime(TDATE *tm);
	static bool ReadOnlyTime(TDATE *tm);
	static bool SetDtTm(const TDATE *tm);
	static bool SetDate(const TDATE *tm);
	static bool SetTime(const TDATE *tm);
};

class DtFilter {
private:
	float mFactor;
	float mState;
	float mEmpty;
	int mNanCnt;
public:
	void init(float afactor);
	void inp(float val);
	float get();
};

class TCrc {
public:
	static void Set(uint8_t *p, int cnt);
	static uint16_t Build(const uint8_t *p, int cnt);
	static uint16_t Proceed(uint16_t Crc, byte uint8_t);
	static bool Check(const uint8_t *p, int cnt);
};

#endif /* UTILS_H_ */
