/*
 * GlobData.h
 *
 *  Created on: 29 gru 2020
 *      Author: Grzegorz
 */

#ifndef GLOBDATA_H_
#define GLOBDATA_H_

#include "cpx.h"
#include "config.h"
#include "msgStream.h"
#include "cmsis_os.h"
#include <DustSensorBase.h>


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
	float tempNTC;
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
	static char *jsonbuf;
	static void Fill();
	static void FillMeas(float *tab);
	static void showDef(MsgStream *strm);
	static void show(MsgStream *strm);
	static void showJson(MsgStream *strm);
	static int buildExportJson();
	static bool getData(float *data);
	static bool isGasMeas(MeasType measTyp);
	static HAL_StatusTypeDef getDustMeas(DustMeasRec *dustMeas);



};

#endif /* GLOBDATA_H_ */