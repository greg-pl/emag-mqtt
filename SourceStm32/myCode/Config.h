/*
 * Config.h
 *
 *  Created on: Dec 7, 2020
 *      Author: Grzegorz
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#include <IOStream.h>
#include "myDef.h"

#define CFG_REC_SIZE 0x800
#define CFG_REC_SIZE_4 (CFG_REC_SIZE/4)

#define CFG_REC_SIZE_PRODUCER 0x80
#define CFG_REC_SIZE_USER (CFG_REC_SIZE - CFG_REC_SIZE_PRODUCER - 0x10)

#define SIZE_SERIAL_NR     12
#define SIZE_DEV_INFO      80
#define SIZE_NET_ADDR      32
#define FACE_LIMIT_TAB_LEN  5

typedef char PathStr[SIZE_NET_ADDR];
typedef char NameStr[32];

typedef enum {
	dustT_SPS30 = 0,  //Siemens
	dustT_HPMA,  	//Honeywell
	dustT_PMSA,  	//Chińczyk
	dustT_PMS5003ST,  	//Chińczyk z Formaldehyde
} DustSensorType;

typedef enum {
	dust_Intern = 0, //
	dust_Extern, //
} DustInpType;

typedef enum {
	gpsOFF = 0, //
	gpsONE_TIME, //
	gpsRUN
} GpsMode;

typedef enum {
	ssUNKNOWN = 0, //
	ssTEMPERATURE, //
	ssHUMIDITY, //
	ssPRESSURE, //
	ssPM1_0, //
	ssPM2_5, //
	ssPM10, //
	ssNO2, //
	ssO3, //
	ssCO, //8
	ssCO2, //8
	ssSO2, //9
	ssCh2o, //10
	ssNOISE, //11
	SENSOR_CNT
} MeasType;

typedef union __PACKED {
	uint8_t tab_b[CFG_REC_SIZE];
	uint32_t tab_32[CFG_REC_SIZE_4];
	struct {
		uint8_t tab_bm4[CFG_REC_SIZE - 4];
		uint16_t free;
		uint16_t Crc;
	};

	struct {
		int Sign;
		uint16_t size;
		uint16_t ver;

		union __PACKED {
			uint8_t buf_p[CFG_REC_SIZE_PRODUCER];
			struct {
				int Sign;
				uint16_t size;
				uint16_t free;
				char SerialNr[SIZE_SERIAL_NR];
				uint8_t dustSensorType;
				uint8_t dustInpType;
			} P;
		};

		union __PACKED {
			uint8_t buf[CFG_REC_SIZE_USER];
			struct {
				union {
					char bg96buf[0x100];
					struct {
						int Sign;
						uint16_t size;
						uint16_t free;
						char DevInfo[SIZE_DEV_INFO];
						TcpInterfDef tcp;
						TDATE rtcSetUpTime;
						int timeZoneShift;
						float gpsLatitude;
						float gpsLongitude;
					};
				};

				union {
					char bg96buf[0x100];
					struct {
						char SimPin[8];
						int Free;
						PathStr ApnName;
						struct {
							PathStr SvrName;
							int WaitTime;
							int RefreshTime; //czas w minutach
						} ntp;
						bool autoStart;
						bool autoOpenMqttSvr;
						int mqttSendInterval; // czas w sekundach między pakietami, 0 - wysyłanie nieaktywne
						int rssiRefreshTime;
						struct {
							int refreshTime; //co ile sekund
							int Mode;  //GpsMode
							bool setRtcTime; //czy ustawiać czas z GPS
						} gps;
						char BgEcho[8];
					};
				} bg96;

				union {
					char mqttBuf[0x200];
					struct {
						PathStr SvrName;
						NameStr userName;
						NameStr password;
						uint16_t SvrPortNoSSL;
						uint16_t SvrPortSSL;
						uint16_t SvrPortPSK;
						bool useSSL;
						bool usePSK;
						int maxLoginTime; //sek
						int keepAlive; //sek
						int qos; //MQTT QOS
						bool retain; //MQTT Retain
						NameStr varNamePub; //nazwa publikowanej zmiennej
						NameStr varNamePub2; //nazwa publikowanej zmiennej
					};
				} mqtt;
				union {
					char restBuf[0xC0];
					struct {
						int free1;
						int free2;
						bool ledOff; // czy wyłaczyć diodę LED na PCB
						int mdb1dbgLevel; //poziom komunikatów na Modbus 1
						int mdb2dbgLevel; //poziom komunikatów na Modbus 2
						int gasDevMdbNr; //numer Modbusa dla komory gazów
						int mdb3dbgLevel; //poziom komunikatów na Modbus 3
						int dustDevMdbNr; //numer Modbusa dla zewnętrznego czujnika pyłów

						bool ledMatrixRun; // czy podłączona tablica led
						bool faceAutoSend; // czy wysyłac automatycznie do LedMatrix
						float faceLimitTab[FACE_LIMIT_TAB_LEN]; // progi do sterownia kodami dla tablicy led
						float faceHistereza; //histereza dla sterowania programi tablicy led
						int faceLevel; //pozion jasności świecenia panelu
					};
				} rest;
				union {
					char restBuf[0x40];
					struct {
						bool sensExist[SENSOR_CNT];
						int gasFiltrType; // 0-OFF, 1-FIR,  2-IR
						int filtrFIRLength;
						float filtrIRConst;
						struct {
							bool useNTCtemp;
							uint8_t showMsg;
							bool runInternal;
							bool runExternal;
							float tempON;
							float tempOFF;
							float humidityON;
							float humidityOFF;
						} heater;
						int noiseFiltrType; // 0-OFF, 1-FIR,  2-IR
						int noiseFiltrFIRLength;
						float noiseFiltrIRConst;
						struct {
							bool humidityEnab;
						} heater2;
					};
				} exDev;
			} R;
		};
	};
} CfgRec;

class Config {
private:
	HAL_StatusTypeDef LoadIntern(uint32_t adr);
	void prepareToSave();
	bool Korekt();
	bool checkRange(float v, float min, float max);
	bool checkRange(int v, int min, int max);
public:
	CfgRec data;
	Config();
	HAL_StatusTypeDef Init(OutStream *strm);
	void Zero();
	void Default();
	void shell(OutStream *strem, const char *cmd);
	bool saveRtc();
	HAL_StatusTypeDef saveFlash();

};

#endif /* CONFIG_H_ */
