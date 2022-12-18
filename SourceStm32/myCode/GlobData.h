/*
 * GlobData.h
 *
 *  Created on: 29 gru 2020
 *      Author: Grzegorz
 */

#ifndef GLOBDATA_H_
#define GLOBDATA_H_

#include "Projectconfig.h"

#include "cpx.h"
#include "config.h"
#include "cmsis_os.h"
#include <DustSensorBase.h>
#include <IOStream.h>
#include <UniDev.h>


typedef struct {
	char code[20];
	float value;
	MeasType measType;
} SensorDt;


typedef struct {
	char version[20];
	char name_space[10];
	char imei[20];
	char serianNr[20];
	float latitude;
	float longitude;
	bool gpsFix;
	int gpsSrc;
	float signalRssi;
	char status[20];
	char info[SIZE_DEV_INFO];
	int pktNr;
	char komoraSt[80];
#if (TEMP_NTC)
	float tempNTC;
#endif
	char hsn[SIZE_SERIAL_NR];

	TDATE time;
	SensorDt sensorDt[SENSOR_CNT];
} GlobDtRec;




class GlobData {
private:
	static osMutexId mGlobMutex;
	static void initMutex();
	static bool openMutex(int tm);
	static void closeMutex();

public:
	enum {
		JSON_SIZE = 2048,
	};

	static GlobDtRec dt;
	static CxString *jsonbuf;
	static const char *GetMeasName(MeasType meas);
	static const char *GetMeasUnit(MeasType meas);
	static const char* GetMeasPrecisionStr(MeasType meas);
	static void Fill();
	static void FillMeas(float *tab);
	static void showDef(OutStream *strm);
	static void show(OutStream *strm);
	static void showJson(OutStream *strm);
	static int buildExportJson();
	static bool getData(float *data);
#if(SENSOR_DUST)
	static UniDev *getDustSensor();
#endif

};

#endif /* GLOBDATA_H_ */
