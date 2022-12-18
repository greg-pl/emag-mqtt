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
#include "cpx.h"
#include "CxString.h"
#include "ProjectConfig.h"


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

extern const CpxFloatDefItem floatGpsLatitudeDef;
extern const CpxFloatDefItem floatGpsLongitudeDef;

typedef enum {
	ssUNKNOWN = 0, //
	ssTEMPERATURE, //
	ssHUMIDITY, //
	ssPRESSURE, //
#if (SENSOR_DUST)
	ssPM1_0, //
	ssPM2_5, //
	ssPM10, //
#endif
	ssNO2, //
	ssO3, //
	ssCO, //8
	ssCO2, //8
	ssSO2, //9
#if(SENSOR_CH_SO)
	ssCh2o, //10
#endif
#if (SENSOR_NOISE)
	ssNOISE, //11
#endif
	SENSOR_CNT
} MeasType;


#if (DEV_DUST_INTERN)
typedef enum {
	dustT_SPS30 = 0,  //Siemens
	dustT_PMSA,  	//Chińczyk
	dustT_PMS5003ST,  	//Chińczyk z Formaldehyde
	dustT_Cnt
} DustSensorType;
#endif

#if (DEV_DUST_INT_EXT)
typedef enum {
	dust_Intern = 0, //
	dust_Extern, //
} DustInpType;
#endif

typedef enum {
	gpsOFF = 0, //
	gpsONE_TIME, //
	gpsRUN
} GpsMode;

#if (LED_MATRIX)
typedef union {
	char buf[0x30];
	struct {
		bool run; // czy podłączona tablica led
		bool autoSend; // czy wysyłac automatycznie do LedMatrix
		float limitTab[FACE_LIMIT_TAB_LEN]; // progi do sterownia kodami dla tablicy led
		float histereza; //histereza dla sterowania programi tablicy led
		int lightLevel; //pozion jasności świecenia panelu
	};
} LedMatrixCfg;
#endif

#if (HEATER)
typedef union {
	char buf[0x30];
	struct {
		bool runInternal;
		bool runExternal;
		bool useNTCtemp;
		bool humidityEnab;
		uint8_t showMsg;
		float tempON;
		float tempOFF;
		float humidityON;
		float humidityOFF;
	};
} HeaterCfg;
#endif

typedef union {
	char mqttBuf[0x200];
	struct {
		bool autoOpenMqttSvr;
		int mqttSendInterval; // czas w sekundach między pakietami, 0 - wysyłanie nieaktywne
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
} MqttCfg;

typedef union {
	char buf[0xC0];
	struct {
		char SerialNr[SIZE_SERIAL_NR];
		char DevInfo[SIZE_DEV_INFO];
		TDATE rtcSetUpTime;
		int timeZoneShift;
		float gpsLatitude;
		float gpsLongitude;
#if (DEV_DUST_INTERN_TYP)
		uint8_t dustSensorIntType;
#endif

#if (DEV_DUST_INT_EXT)
		uint8_t dustInpType;
#endif
		bool pcbLedOff; // czy wyłaczyć diodę LED na PCB
	};
} DevCfg;


typedef union {
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
		int rssiRefreshTime;
		struct {
			int refreshTime; //co ile sekund
			int Mode;  //GpsMode
			bool setRtcTime; //czy ustawiać czas z GPS
		} gps;
		char BgEcho[8];
	};
} Bg96Cfg;

#if (ETHERNET)
typedef union {
	char buf[0x20];
	TcpInterfDef tcp;
} TcpCfg;
#endif

typedef union{
	char buf[0x100];
	struct {
		int mdb1dbgLevel; //poziom komunikatów na Modbus 1
		int mdb2dbgLevel; //poziom komunikatów na Modbus 2
		int gasDevMdbNr; //numer Modbusa dla komory gazów
		int mdb3dbgLevel; //poziom komunikatów na Modbus 3
		int dustDevMdbNr; //numer Modbusa dla zewnętrznego czujnika pyłów
		int gasFiltrType; // 0-OFF, 1-FIR,  2-IR
		int filtrFIRLength;
		float filtrIRConst;
#if(SENSOR_NOISE)
		int noiseFiltrType; // 0-OFF, 1-FIR,  2-IR
		int noiseFiltrFIRLength;
		float noiseFiltrIRConst;
#endif
	};
} RestCfg;


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
			uint8_t buf[CFG_REC_SIZE_USER];
			struct {
				int Sign;
				uint16_t size;
				uint16_t free;
				DevCfg dev;
#if (ETHERNET)
				TcpCfg tcp;
#endif
				Bg96Cfg bg96;
				MqttCfg mqtt;
#if (LED_MATRIX)
				LedMatrixCfg ledMatrix;
#endif
#if (HEATER)
				HeaterCfg heater;
#endif
				bool sensExist[SENSOR_CNT];
				RestCfg rest;
			} R;
		};
	};
} CfgRec;

class Config {
private:
	enum {
		JSON_SIZE = 4000,
	};
	static CxString *jsonbuf;
	HAL_StatusTypeDef LoadIntern(uint32_t adr);
	void prepareToSave();
	bool Korekt();
	bool checkRange(float v, float min, float max);
	bool checkRange(int v, int min, int max);
public:
	CfgRec data;
	Config();
	static void funListUni(OutStream *strm, const char *cmd, void *arg, bool withK);
	static void funList(OutStream *strm, const char *cmd, void *arg);
	static void funListK(OutStream *strm, const char *cmd, void *arg);
	static void funSet(OutStream *strm, const char *cmd, void *arg);
	static void funDefault(OutStream *strm, const char *cmd, void *arg);
	static void funSave(OutStream *strm, const char *cmd, void *arg);
	static void funSaveFlash(OutStream *strm, const char *cmd, void *arg);
	static void funInit(OutStream *strm, const char *cmd, void *arg);
	static void funInitFlash(OutStream *strm, const char *cmd, void *arg);
	static void funShowDef(OutStream *strm, const char *cmd, void *arg);
	static void funShowInfo(OutStream *strm, const char *cmd, void *arg);
	static void funShowJson(OutStream *strm, const char *cmd, void *arg);
	static void funHelp(OutStream *strm, const char *cmd, void *arg);

	HAL_StatusTypeDef Init(OutStream *strm);
	void Zero();
	void Default();
	void shell(OutStream *strem, const char *cmd);
	bool saveRtc();
	HAL_StatusTypeDef saveFlash();

};

#endif /* CONFIG_H_ */
