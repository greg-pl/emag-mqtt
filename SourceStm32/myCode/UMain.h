/*
 * UMain.h
 *
 *  Created on: Dec 5, 2020
 *      Author: Grzegorz
 */

#ifndef UMAIN_H_
#define UMAIN_H_

#include "cmsis_os.h"
#include "myDef.h"

extern EventGroupHandle_t sysEvents;
#define EVENT_TERM_RDY (1<<0)
#define EVENT_CREATE_DEVICES  (1<<1)

#ifdef __cplusplus
extern "C" {
#endif

#include <IOStream.h>

typedef volatile struct {
	uint32_t Sign1;
	union {
		uint8_t buf[0x80 - 8];
		struct {
			int startCnt;
			int itmp1;
			int itmp2;
			int itmp3;
		};
	};
	uint32_t Sign2;
} NIR;

extern NIR nir;
extern int mLedOFF;
extern VerInfo mSoftVer;

extern void uMainCont();
extern void reboot(int tm);
extern void getDevStatusAsTxt(char *buf, int max);

#if (SSD1306)
extern void setLcdScrNr(int nr);
extern void setLcdTime(int time);
#endif
extern void setHeaterMsg(uint8_t show);
extern OutStream* getOutStream();

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

#include <Config.h>
extern Config *config;

#if (SENSOR_NOISE)
#include "Noisedetector.h"
extern NoiseDetector *noiseDet;
#endif

#if (LED_MATRIX)
#include <LedMatrix.h>
extern LedMatrix *ledMatrix;
#endif

#if (DEV_DUST_INTERN)
#include <DustSensorBase.h>
extern DustSensorBase *dustInternSensor;
#endif

#if (DEV_DUST_MDB)
#include "MdbDustSensor.h"
extern ExtDustsensor *dustExternSensor;
#endif

#if (TEMP_NTC)

class NTC {
private:
	enum {
		ADC1_BUFFER_LEN = 64,
	};

	static short int mBuffer[ADC1_BUFFER_LEN];
	static uint32_t mStartTick;
	static uint32_t mDeltaTick;
	static volatile bool mMeasDone;
	static volatile bool mNewMeas;
	static volatile int  mError;

	static float getNtcNap();
	static float liczRez(float u);
	static float liczTemp(float rt);
public:
	static float nap;
	static float rez;  //rezystancja w OM
	static float temp;
	static void StartMeasure(void);
	static void OnConvEnd();
	static void OnConvError(ADC_HandleTypeDef *hadc);
	static bool WaitForMeasEnd(int maxTime);
	static bool isNewMeas();
};
#endif

#endif

#endif /* UMAIN_H_ */
