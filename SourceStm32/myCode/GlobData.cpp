/*
 * GlobData.cpp
 *
 *  Created on: 29 gru 2020
 *      Author: Grzegorz
 */

#include <string.h>
#include <math.h>

#include <GlobData.h>
#include <Bg96Driver.h>
#include <UMain.h>
#include <Cpx.h>
#include <Utils.h>

#include <DustSensorBase.h>
#include <I2cDev.h>
#include <MdbMasterTask.h>
#include "_SensorDrivers.h"

extern Bg96Driver *bg96;
extern SHT35Device *sht35;
extern Bmp338Device *bmp338;
extern DustSensorBase *dustInternSensor;
extern ExtDustsensor *dustExternSensor;
extern MdbMasterTask *mdbMaster_1;
extern MdbMasterTask *mdbMaster_2;
extern GasS873 *gasS873;
extern NoiseDetector *noiseDet;

GlobDtRec GlobData::dt;
osMutexId GlobData::mGlobMutex = NULL;
CxString *GlobData::jsonbuf = NULL;

const char* getFrm2(int idx) {
	return "%.2f";
}
const char* getFrm1(int idx) {
	return "%.1f";
}

CpxFloatDefItem floatRssiDef = { min:-100, //
		max:0, //
		getFrm: getFrm1 //
		};

const char* getItemFormat(int idx) {
	MeasType mt = ssUNKNOWN;
	if (idx >= 0 && idx < SENSOR_CNT)
		mt = GlobData::dt.sensorDt[idx].measType;
	return GlobData::GetMeasPrecisionStr(mt);
}

CpxFloatDefItem floatSensorDef = { min:0, //
		max:0, //
		getFrm: getItemFormat //
		};

const CpxDescr SensorDscr[] = { //
		{ ctype : cpxSTR, ofs: offsetof(SensorDt, code), Name : "code", size:sizeof(SensorDt::code) }, //
				{ ctype : cpxFLOAT, ofs: offsetof(SensorDt, value), Name : "value", size:sizeof(SensorDt::value), exPtr:(const void*) &floatSensorDef }, //
				{ ctype : cpxNULL } };

CpxChildInfo sensorGroupInfo = { //
		itemCnt: SENSOR_CNT, //
				itemSize : sizeof(SensorDt), //
				defs: SensorDscr, //
		};

const CpxDescr GlobDataDscr[] = { //
		{ ctype : cpxSTR, ofs: offsetof(GlobDtRec, version), Name : "version", size:sizeof(GlobDtRec::version) }, //
				{ ctype : cpxSTR, ofs: offsetof(GlobDtRec, name_space), Name : "namespace", size:sizeof(GlobDtRec::name_space) }, //
				{ ctype : cpxSTR, ofs: offsetof(GlobDtRec, imei), Name : "imei", size:sizeof(GlobDtRec::imei) }, //
				{ ctype : cpxSTR, ofs: offsetof(GlobDtRec, serianNr), Name : "ssn", size:sizeof(GlobDtRec::serianNr) }, //
				{ ctype : cpxFLOAT, ofs: offsetof(GlobDtRec, latitude), Name : "latitude", size:sizeof(GlobDtRec::latitude), exPtr :(const void*) &floatGpsLatitudeDef }, //
				{ ctype : cpxFLOAT, ofs: offsetof(GlobDtRec, longitude), Name : "longitude", size:sizeof(GlobDtRec::longitude), exPtr :(const void*) &floatGpsLongitudeDef }, //
				{ ctype : cpxBOOL, ofs: offsetof(GlobDtRec, gpsFix), Name : "gpsFix", size:sizeof(GlobDtRec::gpsFix) }, //
				{ ctype : cpxINT, ofs: offsetof(GlobDtRec, gpsSrc), Name : "gpsSrc", size:sizeof(GlobDtRec::gpsSrc) }, //
				{ ctype : cpxFLOAT, ofs: offsetof(GlobDtRec, signalRssi), Name : "signalRssi", size:sizeof(GlobDtRec::signalRssi), exPtr :(const void*) &floatRssiDef }, //
				{ ctype : cpxSTR, ofs: offsetof(GlobDtRec, status), Name : "status", size:sizeof(GlobDtRec::status) }, //
				{ ctype : cpxSTR, ofs: offsetof(GlobDtRec, info), Name : "info", size:sizeof(GlobDtRec::info) }, //

				{ ctype : cpxINT, ofs: offsetof(GlobDtRec, pktNr), Name : "pktNr", size:sizeof(GlobDtRec::pktNr) }, //
				{ ctype : cpxTIME, ofs: offsetof(GlobDtRec, time), Name : "time", size:sizeof(GlobDtRec::time) }, //
				//{ ctype : cpxSTR, ofs: offsetof(GlobDtRec, komoraSt), Name : "status_komora", size:sizeof(GlobDtRec::komoraSt) }, //
				{ ctype : cpxFLOAT, ofs: offsetof(GlobDtRec, tempNTC), Name : "TempNTC", size:sizeof(GlobDtRec::tempNTC) }, //
				{ ctype : cpxSTR, ofs: offsetof(GlobDtRec, hsn), Name : "hsn", size:sizeof(GlobDtRec::hsn) }, //
				{ ctype : cpxCHILD, ofs: offsetof(GlobDtRec, sensorDt), Name : "sensors", size:(sizeof(GlobDtRec::sensorDt) / sizeof(SensorDt)), exPtr :&sensorGroupInfo }, //
				{ ctype : cpxNULL } };

typedef struct {
	MeasType meas;
	const char *Name;
	const char *Unit;
	const char *Precision;
} SensorName;

const SensorName sensorNameTab[] = { //
		{ ssUNKNOWN, "unknown", "??", "%.0f" }, //
				{ ssTEMPERATURE, "temperature", "*C", "%.2f" }, //
				{ ssHUMIDITY, "humidity", "%", "%.2f" }, //
				{ ssPRESSURE, "pressure", "kPa", "%.1f" }, //
				{ ssPM1_0, "pm1", "ug/m3", "%.1f" }, //
				{ ssPM2_5, "pm2.5", "ug/m3", "%.1f" }, //
				{ ssPM10, "pm10", "ug/m3", "%.1f" }, //
				{ ssNO2, "no2", "xx", "%.2f" }, //
				{ ssO3, "o3", "xx", "%.2f" }, //
				{ ssCO, "co", "ppb", "%.0f" }, //
				{ ssCO2, "co2", "ppm", "%.0f" }, //
				{ ssSO2, "so2", "xx", "%.2f" }, //
				{ ssCh2o, "ch2o", "xx", "%.2f" }, //
				{ ssNOISE, "noise", "dB", "%.1f" } //
		};

const char* GlobData::GetMeasName(MeasType meas) {
	const char *p = "sens??";
	for (int i = 0; i < SENSOR_CNT; i++) {
		if (sensorNameTab[i].meas == meas) {
			p = sensorNameTab[i].Name;
			break;
		}
	}
	return p;
}

const char* GlobData::GetMeasUnit(MeasType meas) {
	const char *p = "??";
	for (int i = 0; i < SENSOR_CNT; i++) {
		if (sensorNameTab[i].meas == meas) {
			p = sensorNameTab[i].Unit;
			break;
		}
	}
	return p;
}

const char* GlobData::GetMeasPrecisionStr(MeasType meas) {
	const char *p = "%.1f";
	for (int i = 0; i < SENSOR_CNT; i++) {
		if (sensorNameTab[i].meas == meas) {
			p = sensorNameTab[i].Precision;
			break;
		}
	}
	return p;
}

void GlobData::initMutex() {
	osMutexDef(GlobDt);
	mGlobMutex = osMutexCreate(osMutex(GlobDt));
}

bool GlobData::openMutex(int tm) {
	if (mGlobMutex == NULL)
		initMutex();
	return (osMutexWait(mGlobMutex, tm) == osOK);
}
void GlobData::closeMutex() {
	osMutexRelease(mGlobMutex);
}

void GlobData::FillMeas(float *tab) {

	for (int i = 0; i < SENSOR_CNT; i++) {
		tab[i] = NAN;
	}

	float temperature1 = NAN, humidity = NAN;
	float temperature2 = NAN, pressure = NAN;

	if (sht35 != NULL) {
		sht35->getMeasValue(ssTEMPERATURE, &temperature1);
		sht35->getMeasValue(ssHUMIDITY, &humidity);
	}
	if (bmp338 != NULL) {
		bmp338->getMeasValue(ssTEMPERATURE, &temperature2);
		bmp338->getMeasValue(ssPRESSURE, &pressure);
	}

	tab[ssTEMPERATURE] = temperature1;
	if (isnan(temperature1))
		tab[ssTEMPERATURE] = temperature2;

	tab[ssHUMIDITY] = humidity;
	tab[ssPRESSURE] = pressure;

	UniDev *dustDev = getDustSensor();
	dustDev->getMeasValue(ssPM1_0, &tab[ssPM1_0]);
	dustDev->getMeasValue(ssPM2_5, &tab[ssPM2_5]);
	dustDev->getMeasValue(ssPM10, &tab[ssPM10]);
	dustDev->getMeasValue(ssCh2o, &tab[ssCh2o]);

	if (mdbMaster_2 != NULL) {
		//gas
		gasS873->getMeasValue(ssNO2, &tab[ssNO2]);
		gasS873->getMeasValue(ssO3, &tab[ssO3]);
		gasS873->getMeasValue(ssCO, &tab[ssCO]);
		gasS873->getMeasValue(ssCO2, &tab[ssCO2]);
		gasS873->getMeasValue(ssSO2, &tab[ssSO2]);
	}

	//noise
	if (noiseDet != NULL) {
		noiseDet->getMeasValue(ssNOISE, &tab[ssNOISE]);
	}

}

void GlobData::Fill() {
	if (openMutex(5)) {
		memset(&dt, 0, sizeof(dt));

		snprintf(dt.version, sizeof(dt.version), "1.%u.%u", mSoftVer.ver, mSoftVer.rev);
		strcpy(dt.name_space, "im40");

		strncpy(dt.imei, bg96->bgParam.imei, sizeof(dt.imei));
		strncpy(dt.serianNr, bg96->bgParam.sim_imsi, sizeof(dt.serianNr));
		if (bg96->state.gps.gpsFix) {
			dt.gpsFix = true;
			dt.gpsSrc = 1;
			dt.latitude = bg96->state.gps.latitude;
			dt.longitude = bg96->state.gps.longitude;
		} else if (config->data.R.dev.gpsLatitude != 0 || config->data.R.dev.gpsLongitude != 0) {
			dt.gpsFix = true;
			dt.gpsSrc = 2;
			dt.latitude = config->data.R.dev.gpsLatitude;
			dt.longitude = config->data.R.dev.gpsLongitude;
		} else {
			dt.gpsFix = false;
			dt.gpsSrc = 0;
			dt.latitude = NAN;
			dt.longitude = NAN;
		}
		dt.signalRssi = bg96->state.rssi.rssi;
		getDevStatusAsTxt(dt.status, sizeof(dt.status));
		strncpy(dt.info, config->data.R.dev.DevInfo, sizeof(dt.info));
		dt.pktNr = bg96->state.mqtt.mSendMsgID;
		Rtc::ReadTime(&dt.time);
		gasS873->getDeviceStatusTxt(dt.komoraSt, sizeof(dt.komoraSt));
		dt.tempNTC = NTC::temp;
		strncpy(dt.hsn, config->data.R.dev.SerialNr, sizeof(dt.hsn));

		//pobranie danych pomiarowych
		float tab[SENSOR_CNT];
		FillMeas(tab);
		int k = 0;
		for (int i = 0; i < SENSOR_CNT; i++) {
			bool exist = config->data.R.sensExist[i];
			if (i == ssCh2o) { //Formaldehyde
				exist &= ((config->data.R.dev.dustInpType == dust_Intern) && (config->data.R.dev.dustSensorType == dustT_PMS5003ST));
			}

			if (exist) {
				dt.sensorDt[k].measType = (MeasType) i;
				strlcpy(dt.sensorDt[k].code, GetMeasName((MeasType) i), sizeof(SensorDt::code));
				dt.sensorDt[k].value = tab[i];
				k++;
			}
		}
		//jesli nie ma żadnego czujnika to dodajemy temperature
		if (k == 0) {
			dt.sensorDt[k].measType = ssTEMPERATURE;
			strlcpy(dt.sensorDt[k].code, GetMeasName(ssTEMPERATURE), sizeof(SensorDt::code));
			dt.sensorDt[0].value = tab[ssTEMPERATURE];
			k = 1;
		}
		sensorGroupInfo.itemCnt = k;
		closeMutex();
	} else {
		//jeśli mutex był zajęty to zanczy że, inny task wypełnił rekord
		while (!openMutex(50)) {

		}
		closeMutex();
	}

}

void GlobData::showDef(OutStream *strm) {
	Cpx cpx;
	cpx.init(GlobDataDscr, &dt);
	cpx.showDef(strm);
}

void GlobData::show(OutStream *strm) {
	Fill();
	Cpx cpx;
	cpx.init(GlobDataDscr, &dt);
	cpx.list(strm);
}

void GlobData::showJson(OutStream *strm) {
	if (jsonbuf == NULL) {
		jsonbuf = new CxString(JSON_SIZE);
	}
	if (jsonbuf != NULL) {
		jsonbuf->clear();
		Fill();
		Cpx cpx;
		cpx.init(GlobDataDscr, &dt);
		cpx.buildjson(jsonbuf);
		strm->oMsgX(colWHITE, "Len=%d", jsonbuf->len());
		strm->oBufX(colYELLOW, jsonbuf->p(), jsonbuf->len());
	}
}

int GlobData::buildExportJson() {
	if (jsonbuf == NULL) {
		jsonbuf = new CxString(JSON_SIZE);
	}
	if (jsonbuf != NULL) {
		jsonbuf->clear();
		Fill();
		Cpx cpx;
		cpx.init(GlobDataDscr, &dt);
		cpx.buildjson(jsonbuf);
		jsonbuf->add(CTRL_Z_CH); // potrzebne do wysłania przez BG96
		return jsonbuf->len();
	} else
		return -1;
}

UniDev* GlobData::getDustSensor() {
	if (config->data.R.dev.dustInpType == dust_Intern) {
		return dustInternSensor;
	} else {
		return dustExternSensor;
	}

}

/*
 {
 "version": "1.1.138",
 "namespace": "im40",
 "imei": "866770056670574",
 "ssn": "260060036640920",
 "gpsFix": false,
 "gpsSrc": 0,
 "signalRssi": -71.0,
 "status": "OK",
 "info": "",
 "pktNr": 19,
 "time": "2022.12.12 20:10:33",
 "TempNTC": 30.425037,
 "hsn": "W00001",
 "sensors": [
 {
 "code": "temperature"
 },
 {
 "code": "humidity"
 },
 {
 "code": "pressure",
 "value": 980.4
 },
 {
 "code": "co",
 "value": 1187
 },
 {
 "code": "co2",
 "value": 900
 }
 ]
 }

 */
