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

extern Bg96Driver *bg96;
extern SHT35DevPub *sht35;
extern Bmp338DevPub *bmp338;
extern DustSensorBase *dustInternSensor;
extern MdbMasterDustTask *dustExternSensor;
extern MdbMasterNoiseTask *mdbMaster_1;
extern MdbMasterGasTask *mdbMaster_2;

GlobDtRec GlobData::dt;
osMutexId GlobData::mGlobMutex = NULL;
char *GlobData::jsonbuf = NULL;

extern "C" const char* getGpsFormat(int idx) {
	return "%.5f";
}

const char* getFrm2(int idx) {
	return "%.2f";
}
const char* getFrm1(int idx) {
	return "%.1f";
}

const char* getItemFormat(int idx) {
	if (idx >= 0 && idx < SENSOR_CNT) {
		MeasType mt = GlobData::dt.sensorDt[idx].measType;

		switch (mt) {
		case ssTEMPERATURE:
		case ssNO2: //no2
		case ssO3: //o3
		case ssCO: //co
		case ssSO2: //so2
			return "%.2f";
		case ssCh2o: //Formalhedyte
		case ssPRESSURE:
			return "%.3f";
		case ssHUMIDITY:
		case ssPM1_0:
		case ssPM2_5:
		case ssPM10:
		case ssNOISE:
			return "%.1f";
		default:
			return "%.0f";
		}
	} else
		return "%.0f";
}

const CpxDef SensorDscr[] = { //
		{ ctype : cpxSTR, ofs: offsetof(SensorDt, code), Name : "code", size:sizeof(SensorDt::code) }, //
				{ ctype : cpxFLOAT, ofs: offsetof(SensorDt, value), Name : "value", size:sizeof(SensorDt::value), exPtr:(const void*) getItemFormat }, //
				{ ctype : cpxNULL } };

GroupInfo sensorGroupInfo = { //
		itemCnt: SENSOR_CNT, //
				itemSize : sizeof(SensorDt), //
				defs: SensorDscr, //
		};

const CpxDef GlobDataDscr[] = { //
		{ ctype : cpxSTR, ofs: offsetof(GlobDtRec, version), Name : "version", size:sizeof(GlobDtRec::version) }, //
				{ ctype : cpxSTR, ofs: offsetof(GlobDtRec, name_space), Name : "namespace", size:sizeof(GlobDtRec::name_space) }, //
				{ ctype : cpxSTR, ofs: offsetof(GlobDtRec, imei), Name : "imei", size:sizeof(GlobDtRec::imei) }, //
				{ ctype : cpxSTR, ofs: offsetof(GlobDtRec, serianNr), Name : "ssn", size:sizeof(GlobDtRec::serianNr) }, //
				{ ctype : cpxFLOAT, ofs: offsetof(GlobDtRec, latitude), Name : "latitude", size:sizeof(GlobDtRec::latitude), exPtr :(const void*) getGpsFormat }, //
				{ ctype : cpxFLOAT, ofs: offsetof(GlobDtRec, longitude), Name : "longitude", size:sizeof(GlobDtRec::longitude), exPtr :(const void*) getGpsFormat }, //
				{ ctype : cpxBOOL, ofs: offsetof(GlobDtRec, gpsFix), Name : "gpsFix", size:sizeof(GlobDtRec::gpsFix) }, //
				{ ctype : cpxINT, ofs: offsetof(GlobDtRec, gpsSrc), Name : "gpsSrc", size:sizeof(GlobDtRec::gpsSrc) }, //
				{ ctype : cpxFLOAT, ofs: offsetof(GlobDtRec, signalRssi), Name : "signalRssi", size:sizeof(GlobDtRec::signalRssi), exPtr :(const void*) getFrm1 }, //
				{ ctype : cpxSTR, ofs: offsetof(GlobDtRec, status), Name : "status", size:sizeof(GlobDtRec::status) }, //
				{ ctype : cpxSTR, ofs: offsetof(GlobDtRec, info), Name : "info", size:sizeof(GlobDtRec::info) }, //

				{ ctype : cpxINT, ofs: offsetof(GlobDtRec, pktNr), Name : "pktNr", size:sizeof(GlobDtRec::pktNr) }, //
				{ ctype : cpxTIME, ofs: offsetof(GlobDtRec, time), Name : "time", size:sizeof(GlobDtRec::time) }, //
				//{ ctype : cpxSTR, ofs: offsetof(GlobDtRec, komoraSt), Name : "status_komora", size:sizeof(GlobDtRec::komoraSt) }, //
				{ ctype : cpxFLOAT, ofs: offsetof(GlobDtRec, tempNTC), Name : "TempNTC", size:sizeof(GlobDtRec::tempNTC) }, //
				{ ctype : cpxSTR, ofs: offsetof(GlobDtRec, hsn), Name : "hsn", size:sizeof(GlobDtRec::hsn) }, //
				{ ctype : cpxTAB, ofs: offsetof(GlobDtRec, sensorDt), Name : "sensors", size:(sizeof(GlobDtRec::sensorDt) / sizeof(SensorDt)), exPtr :&sensorGroupInfo }, //
				{ ctype : cpxNULL } };

const char *const sensorNameTab[SENSOR_CNT] = { "temperature", "humidity", "pressure", "pm1", "pm2.5", "pm10", "no2", "o3", "co", "so2", "ch2o", "noise" };

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
	float temperature1, humidity;
	float temperature2, pressure;
	sht35->getData(&temperature1, &humidity);
	bmp338->getData(&temperature2, &pressure);

	tab[ssTEMPERATURE] = temperature1;
	tab[ssHUMIDITY] = humidity;
	tab[ssPRESSURE] = pressure;

	//dust
	DustMeasRec dustMeas;

	HAL_StatusTypeDef dustSt = getDustMeas(&dustMeas);
	if (dustSt == HAL_OK) {
		tab[ssPM1_0] = dustMeas.pm1_0;
		tab[ssPM2_5] = dustMeas.pm2_5;
		tab[ssPM10] = dustMeas.pm10;
		tab[ssCh2o] = dustMeas.Formaldehyde;
	} else {
		tab[ssPM1_0] = NAN;
		tab[ssPM2_5] = NAN;
		tab[ssPM10] = NAN;
		tab[ssCh2o] = NAN;
	}

	float val;
	//gas
	mdbMaster_2->getGasValue(ssNO2, &val);
	tab[ssNO2] = val;
	mdbMaster_2->getGasValue(ssO3, &val);
	tab[ssO3] = val;
	mdbMaster_2->getGasValue(ssCO, &val);
	tab[ssCO] = val;
	mdbMaster_2->getGasValue(ssSO2, &val);
	tab[ssSO2] = val;
	//noise
	mdbMaster_1->getNoiseValue(&val);
	tab[ssNOISE] = val;

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
		} else if (config->data.R.gpsLatitude != 0 || config->data.R.gpsLongitude != 0) {
			dt.gpsFix = true;
			dt.gpsSrc = 2;
			dt.latitude = config->data.R.gpsLatitude;
			dt.longitude = config->data.R.gpsLongitude;
		} else {
			dt.gpsFix = false;
			dt.gpsSrc = 0;
			dt.latitude = NAN;
			dt.longitude = NAN;
		}
		dt.signalRssi = bg96->state.rssi.rssi;
		getDevStatusAsTxt(dt.status, sizeof(dt.status));
		strncpy(dt.info, config->data.R.DevInfo, sizeof(dt.info));
		dt.pktNr = bg96->state.mqtt.mSendMsgID;
		Rtc::ReadTime(&dt.time);
		mdbMaster_2->getKomoraStatusTxt(dt.komoraSt, sizeof(dt.komoraSt));
		dt.tempNTC = NTC::temp;
		strncpy(dt.hsn, config->data.P.SerialNr, sizeof(dt.hsn));


		//pobranie danych pomiarowych
		float tab[SENSOR_CNT];
		FillMeas(tab);
		int k = 0;
		for (int i = 0; i < SENSOR_CNT; i++) {
			bool exist = config->data.R.exDev.sensExist[i];
			if (i == ssCh2o) { //Formaldehyde
				exist &= ((config->data.P.dustInpType == dust_Intern) && (config->data.P.dustSensorType == dustT_PMS5003ST));
			}

			if (exist) {
				strncpy(dt.sensorDt[k].code, sensorNameTab[i], sizeof(SensorDt::code));
				dt.sensorDt[k].value = tab[i];
				dt.sensorDt[k].measType = (MeasType) i;
				k++;
			}
		}
		//jesli nie ma żadnego czujnika to dodajemy temperature
		if (k == 0) {
			strncpy(dt.sensorDt[0].code, sensorNameTab[ssTEMPERATURE], sizeof(SensorDt::code));
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

void GlobData::showDef(MsgStream *strm) {
	Cpx cpx;
	cpx.init(GlobDataDscr, &dt);
	cpx.showDef(strm);
}

void GlobData::show(MsgStream *strm) {
	Fill();
	Cpx cpx;
	cpx.init(GlobDataDscr, &dt);
	cpx.list(strm);

}

void GlobData::showJson(MsgStream *strm) {
	if (jsonbuf == NULL) {
		jsonbuf = (char*) malloc(JSON_SIZE);
	}
	if (jsonbuf != NULL) {
		Fill();
		Cpx cpx;
		cpx.init(GlobDataDscr, &dt);
		int len0 = cpx.buildjson(jsonbuf, JSON_SIZE);
		int len = strlen(jsonbuf);
		strm->msg(colWHITE, "Len=%d %d", len, len0);
		strm->dumpBuf(colYELLOW, jsonbuf);
	}
}

int GlobData::buildExportJson() {
	if (jsonbuf == NULL) {
		jsonbuf = (char*) malloc(JSON_SIZE);
	}
	if (jsonbuf != NULL) {
		Fill();
		Cpx cpx;
		cpx.init(GlobDataDscr, &dt);
		cpx.buildjson(jsonbuf, JSON_SIZE);
		int len = strlen(jsonbuf);
		if (len > 0) {
			jsonbuf[len++] = CTRL_Z_CH; // potrzebne do wysłania przez BG96
			jsonbuf[len] = 0;
		}
		return len;
	} else
		return -1;
}

bool GlobData::isGasMeas(MeasType measTyp) {
	return (measTyp == ssNO2) || (measTyp == ssO3) || (measTyp == ssCO) || (measTyp == ssSO2);
}

HAL_StatusTypeDef GlobData::getDustMeas(DustMeasRec *dustMeas) {
	if (config->data.P.dustInpType == dust_Intern) {
		return dustInternSensor->getMeas(dustMeas);
	} else {
		return dustExternSensor->getMeas(dustMeas);
	}
}