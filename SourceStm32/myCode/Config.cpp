/*
 * Config.cpp
 *
 *  Created on: Dec 7, 2020
 *      Author: Grzegorz
 */

#include "string.h"
#include "stdio.h"

#include "stm32f4xx_hal.h"

#include "main.h"
#include <Utils.h>
#include <Cpx.h>
#include <MdbCrc.h>
#include <myDef.h>

#include <Config.h>
#include <ShellItem.h>
#include <Token.h>

extern Config *config;
extern IWDG_HandleTypeDef hiwdg;

//export do jezyka C

#if (ETHERNET)
extern "C" const TcpInterfDef* getTcpDef() {
	static TcpInterfDef def;
	memcpy(&def, &config->data.R.tcp, sizeof(def));
	return &def;
}
#endif

#define  CFG_SIGN  0x2367A3B7
#define  CFG_SIGN_P  0x2367A3B2
#define  CFG_SIGN_U  0x2367A3BB

#define CFG_ADDR_RTCRAM   (BKPSRAM_BASE+0)
#define CFG_ADDR_FLASH    0x080E0000

extern "C" const char* getGpsFormat(int idx) {
	return "%.5f";
}

const CpxFloatDefItem floatGpsLatitudeDef = { min:-90, //
		max:90, //
		getFrm: getGpsFormat //
		};

const CpxFloatDefItem floatGpsLongitudeDef = { min:-180, //
		max:180, //
		getFrm: getGpsFormat //
		};

const CpxDescr SensorOnOffDscr[] = { //
		{ ctype : cpxBOOL, id:1, ofs: ssTEMPERATURE, Name : "existSensTemper", size : sizeof(bool) }, //
				{ ctype : cpxBOOL, id:2, ofs: ssHUMIDITY, Name : "existSensHumidity", size: sizeof(bool) }, //
				{ ctype : cpxBOOL, id:3, ofs: ssPRESSURE, Name : "existSensPressure", size: sizeof(bool) }, //
#if(SENSOR_DUST)
				{ ctype : cpxBOOL, id:4, ofs: ssPM1_0, Name : "existSensPM1_0", size: sizeof(bool) }, //
				{ ctype : cpxBOOL, id:5, ofs: ssPM2_5, Name : "existSensPM2_5", size: sizeof(bool) }, //
				{ ctype : cpxBOOL, id:6, ofs: ssPM10, Name : "existSensPM10", size : sizeof(bool) }, //
#endif
				{ ctype : cpxBOOL, id:7, ofs: ssNO2, Name : "existSensNO2", size: sizeof(bool) }, //
				{ ctype : cpxBOOL, id:8, ofs: ssO3, Name : "existSensO3", size: sizeof(bool) }, //
				{ ctype : cpxBOOL, id:9, ofs: ssCO, Name : "existSensCO", size: sizeof(bool) }, //
				{ ctype : cpxBOOL, id:10, ofs: ssCO2, Name : "existSensCO2", size: sizeof(bool) }, //
				{ ctype : cpxBOOL, id:11, ofs: ssSO2, Name : "existSensSO2", size: sizeof(bool) }, //
#if(SENSOR_CH_SO)
				{ ctype : cpxBOOL, id:12, ofs: ssCh2o, Name : "existSensCH2O", size: sizeof(bool) }, //
#endif
#if (SENSOR_NOISE)

				{ ctype : cpxBOOL, id:13, ofs: ssNOISE, Name : "existSensNoise", size: sizeof(bool) }, //
#endif
				{ ctype : cpxNULL } };

const CpxChildInfo sensorOnOffGroup = { //
		itemCnt: 1, //
				itemSize : sizeof(CfgRec::R.sensExist), //
				defs: SensorOnOffDscr, //
		};
#if (LED_MATRIX)
const CpxDescr LedMatrixDescr[] = { //

		{ ctype : cpxBOOL, id:1, ofs: offsetof(LedMatrixCfg, run), Name : "run", size : sizeof(LedMatrixCfg::run) }, //
				{ ctype : cpxBOOL, id:2, ofs: offsetof(LedMatrixCfg, autoSend), Name : "auto", size : sizeof(LedMatrixCfg::autoSend) }, //
				{ ctype : cpxINT, id:3, ofs: offsetof(LedMatrixCfg, lightLevel), Name : "level", size : sizeof(LedMatrixCfg::lightLevel) }, //
				{ ctype : cpxFLOAT, id:4, ofs: offsetof(LedMatrixCfg, histereza), Name : "histereza", size : sizeof(LedMatrixCfg::histereza) }, //
				{ ctype : cpxFLOAT, id:5, ofs: offsetof(LedMatrixCfg, limitTab[0]), Name : "lev1", size : sizeof(LedMatrixCfg::limitTab[0]) }, //
				{ ctype : cpxFLOAT, id:6, ofs: offsetof(LedMatrixCfg, limitTab[1]), Name : "lev2", size : sizeof(LedMatrixCfg::limitTab[1]) }, //
				{ ctype : cpxFLOAT, id:7, ofs: offsetof(LedMatrixCfg, limitTab[2]), Name : "lev3", size : sizeof(LedMatrixCfg::limitTab[2]) }, //
				{ ctype : cpxFLOAT, id:8, ofs: offsetof(LedMatrixCfg, limitTab[3]), Name : "lev4", size : sizeof(LedMatrixCfg::limitTab[3]) }, //
				{ ctype : cpxFLOAT, id:9, ofs: offsetof(LedMatrixCfg, limitTab[4]), Name : "lev5", size : sizeof(LedMatrixCfg::limitTab[4]) }, //
				{ ctype : cpxNULL } };

const CpxChildInfo ledMatrixGroup = { //
		itemCnt: 1, //
				itemSize : sizeof(LedMatrixCfg), //
				defs: LedMatrixDescr, //
		};
#endif

#if (HEATER)

const CpxDescr HeaterDescr[] = { //

		{ ctype : cpxBOOL, id:1, ofs: offsetof(HeaterCfg, useNTCtemp), Name : "useNtcTemp", size: sizeof(bool) }, //
				{ ctype : cpxBOOL, id:2, ofs: offsetof(HeaterCfg, runExternal), Name : "runExternal", size: sizeof(bool) }, //
				{ ctype : cpxBOOL, id:3, ofs: offsetof(HeaterCfg, runInternal), Name : "runInternal", size: sizeof(bool) }, //
				{ ctype : cpxBYTE, id:4, ofs: offsetof(HeaterCfg, showMsg), Name : "showMsg", size: sizeof(bool) }, //
				{ ctype : cpxFLOAT, id:5, ofs: offsetof(HeaterCfg, tempON), Name : "tempON", size : sizeof(HeaterCfg::tempON) }, //
				{ ctype : cpxFLOAT, id:6, ofs: offsetof(HeaterCfg, tempOFF), Name : "tempOFF", size : sizeof(HeaterCfg::tempOFF) }, //
				{ ctype : cpxFLOAT, id:7, ofs: offsetof(HeaterCfg, humidityON), Name : "humidityON", size : sizeof(HeaterCfg::humidityON) }, //
				{ ctype : cpxFLOAT, id:8, ofs: offsetof(HeaterCfg, humidityOFF), Name : "humidityOFF", size : sizeof(HeaterCfg::humidityOFF) }, //
				{ ctype : cpxBOOL, id:9, ofs: offsetof(HeaterCfg, humidityEnab), Name : "humidityENAB", size : sizeof(HeaterCfg::humidityEnab) }, //
				{ ctype : cpxNULL } };

const CpxChildInfo heaterGroup = { //
		itemCnt: 1, //
				itemSize : sizeof(HeaterCfg), //
				defs: HeaterDescr, //
		};
#endif

const CpxDescr MqttDescr[] = { //
		{ ctype : cpxBOOL, id:1, ofs: offsetof(MqttCfg, autoOpenMqttSvr), Name : "AutoOpenSvr", size : sizeof(MqttCfg::autoOpenMqttSvr) }, //
				{ ctype : cpxINT, id:2, ofs: offsetof(MqttCfg, mqttSendInterval), Name : "SendInterval", size : sizeof(MqttCfg::mqttSendInterval) }, //
				{ ctype : cpxSTR, id:3, ofs: offsetof(MqttCfg, SvrName), Name : "SvrName", size: sizeof(MqttCfg::SvrName) }, //
				{ ctype : cpxWORD, id:4, ofs: offsetof(MqttCfg, SvrPortNoSSL), Name : "PortNoSSL", size : sizeof(MqttCfg::SvrPortNoSSL) }, //
				{ ctype : cpxWORD, id:5, ofs: offsetof(MqttCfg, SvrPortSSL), Name : "PortSSL", size: sizeof(MqttCfg::SvrPortSSL) }, //
				{ ctype : cpxWORD, id:6, ofs: offsetof(MqttCfg, SvrPortPSK), Name : "PortPSK", size: sizeof(MqttCfg::SvrPortPSK) }, //
				{ ctype : cpxSTR, id:7, ofs: offsetof(MqttCfg, userName), Name : "UserName", size: sizeof(MqttCfg::userName) }, //
				{ ctype : cpxSTR, id:8, ofs: offsetof(MqttCfg, password), Name : "UserPassword", size : sizeof(MqttCfg::password) }, //
				{ ctype : cpxBOOL, id:9, ofs: offsetof(MqttCfg, useSSL), Name : "UseSSL", size: sizeof(MqttCfg::useSSL) }, //
				{ ctype : cpxBOOL, id:10, ofs: offsetof(MqttCfg, usePSK), Name : "UsePSK", size: sizeof(MqttCfg::usePSK) }, //
				{ ctype : cpxINT, id:11, ofs: offsetof(MqttCfg, maxLoginTime), Name : "MaxLoginTime", size : sizeof(MqttCfg::maxLoginTime) }, //
				{ ctype : cpxINT, id:12, ofs: offsetof(MqttCfg, keepAlive), Name : "KeepAlive", size : sizeof(MqttCfg::keepAlive) }, //
				{ ctype : cpxINT, id:13, ofs: offsetof(MqttCfg, qos), Name : "QOS", size: sizeof(MqttCfg::qos) }, //
				{ ctype : cpxBOOL, id:14, ofs: offsetof(MqttCfg, retain), Name : "Retain", size: sizeof(MqttCfg::retain) }, //
				{ ctype : cpxSTR, id:15, ofs: offsetof(MqttCfg, varNamePub), Name : "SendVarName", size : sizeof(MqttCfg::varNamePub) }, //
				{ ctype : cpxSTR, id:16, ofs: offsetof(MqttCfg, varNamePub2), Name : "SendVar2Name", size : sizeof(MqttCfg::varNamePub2) }, //
				{ ctype : cpxNULL } };

const CpxChildInfo mqttGroup = { //
		itemCnt: 1, //
				itemSize : sizeof(MqttCfg), //
				defs: MqttDescr, //
		};

const CpxDescr DevInfoDescr[] = { //

		{ ctype : cpxSTR, id:1, ofs: offsetof(DevCfg, SerialNr), Name : "SerialNr", size : sizeof(DevCfg::SerialNr) }, //
				{ ctype : cpxSTR, id:2, ofs: offsetof(DevCfg, DevInfo), Name : "Info", size : sizeof(DevCfg::DevInfo) }, //
				{ ctype : cpxINT, id:3, ofs: offsetof(DevCfg, timeZoneShift), Name : "TimeZoneShift", size : sizeof(DevCfg::timeZoneShift) }, //
				{ ctype : cpxFLOAT, id:4, ofs: offsetof(DevCfg, gpsLatitude), Name : "GpsLatitude", size : sizeof(DevCfg::gpsLatitude), exPtr :(const void*) &floatGpsLatitudeDef }, //
				{ ctype : cpxFLOAT, id:5, ofs: offsetof(DevCfg, gpsLongitude), Name : "GpsLongitude", size : sizeof(DevCfg::gpsLongitude), exPtr :(const void*) &floatGpsLongitudeDef }, //
#if (DEV_DUST_INTERN_TYP)
				{ ctype : cpxBYTE, id:7, ofs: offsetof(DevCfg, dustSensorIntType), Name : "DustSensorType" }, //
#endif
#if (DEV_DUST_INT_EXT)
				{ ctype : cpxBYTE, id:6, ofs: offsetof(DevCfg, dustInpType), Name : "DustInpType" }, //
#endif
				{ ctype : cpxBOOL, id:8, ofs: offsetof(DevCfg, pcbLedOff), Name : "PcbLedOff", size: sizeof(DevCfg::pcbLedOff) }, //

				{ ctype : cpxNULL } };

const CpxChildInfo devInfoGroup = { //
		itemCnt: 1, //
				itemSize : sizeof(DevCfg), //
				defs: DevInfoDescr, //
		};

#if (ETHERNET)
const CpxDescr TcpDescr[] = { //
		{ ctype : cpxBYTE, id:1, ofs: offsetof(TcpInterfDef, dhcp), Name : "dhcp", size: sizeof(TcpInterfDef::dhcp) }, //
				{ ctype : cpxIP, id:2, ofs: offsetof(TcpInterfDef, ip), Name : "ip", size: sizeof(TcpInterfDef::ip) }, //
				{ ctype : cpxIP, id:3, ofs: offsetof(TcpInterfDef, mask), Name : "mask", size: sizeof(TcpInterfDef::mask) }, //
				{ ctype : cpxIP, id:4, ofs: offsetof(TcpInterfDef, gw), Name : "gw", size: sizeof(TcpInterfDef::gw) }, //
				{ ctype : cpxIP, id:5, ofs: offsetof(TcpInterfDef, dns1), Name : "dns1", size: sizeof(TcpInterfDef::dns1) }, //
				{ ctype : cpxIP, id:6, ofs: offsetof(TcpInterfDef, dns2), Name : "dns2", size: sizeof(TcpInterfDef::dns2) }, //

				{ ctype : cpxNULL } };

const CpxChildInfo tcpGroup = { //
		itemCnt: 1, //
				itemSize : sizeof(TcpCfg), //
				defs: TcpDescr, //
		};
#endif

const CpxDescr BgDescr[] = { //
		{ ctype : cpxBOOL, id:1, ofs: offsetof(Bg96Cfg, autoStart), Name : "AutoStart", size: sizeof(Bg96Cfg::autoStart) }, //
				{ ctype : cpxSTR, id:2, ofs: offsetof(Bg96Cfg, BgEcho), Name : "BgEcho", size: sizeof(Bg96Cfg::BgEcho) }, //
				{ ctype : cpxSTR, id:3, ofs: offsetof(Bg96Cfg, SimPin), Name : "SimPin", size: sizeof(Bg96Cfg::SimPin) }, //
				{ ctype : cpxSTR, id:4, ofs: offsetof(Bg96Cfg, ApnName), Name : "ApnName", size: sizeof(Bg96Cfg::ApnName) }, //
				{ ctype : cpxINT, id:5, ofs: offsetof(Bg96Cfg, rssiRefreshTime), Name : "RssiRefreshTime", size : sizeof(Bg96Cfg::rssiRefreshTime) }, //
				{ ctype : cpxINT, id:6, ofs: offsetof(Bg96Cfg, gps.Mode), Name : "GpsMode", size: sizeof(Bg96Cfg::gps.Mode) }, //
				{ ctype : cpxINT, id:7, ofs: offsetof(Bg96Cfg, gps.refreshTime), Name : "GpsRefreshTime", size : sizeof(Bg96Cfg::gps.refreshTime) }, //
				{ ctype : cpxBOOL, id:8, ofs: offsetof(Bg96Cfg, gps.setRtcTime), Name : "GpsSetRtcTime", size : sizeof(Bg96Cfg::gps.setRtcTime) }, //
				{ ctype : cpxSTR, id:9, ofs: offsetof(Bg96Cfg, ntp.SvrName), Name : "NtpSvrName", size: sizeof(Bg96Cfg::ntp.SvrName) }, //
				{ ctype : cpxINT, id:10, ofs: offsetof(Bg96Cfg, ntp.WaitTime), Name : "NtpWaitTime", size : sizeof(Bg96Cfg::ntp.WaitTime) }, //
				{ ctype : cpxINT, id:11, ofs: offsetof(Bg96Cfg, ntp.RefreshTime), Name : "NtpRefreshTime", size : sizeof(Bg96Cfg::ntp.RefreshTime) }, //
				{ ctype : cpxNULL } };

const CpxChildInfo bgGroup = { //
		itemCnt: 1, //
				itemSize : sizeof(Bg96Cfg), //
				defs: BgDescr, //
		};

const CpxDescr RestCfgDescr[] = { //
		{ ctype : cpxINT, id:1, ofs: offsetof(RestCfg, mdb1dbgLevel), Name : "Mdb1DbgLevel", size : sizeof(RestCfg::mdb1dbgLevel) }, //
				{ ctype : cpxINT, id:2, ofs: offsetof(RestCfg, mdb2dbgLevel), Name : "Mdb2DbgLevel", size : sizeof(RestCfg::mdb2dbgLevel) }, //
				{ ctype : cpxINT, id:3, ofs: offsetof(RestCfg, mdb3dbgLevel), Name : "Mdb3DbgLevel", size : sizeof(RestCfg::mdb3dbgLevel) }, //
				{ ctype : cpxINT, id:4, ofs: offsetof(RestCfg, gasDevMdbNr), Name : "GasDevMdbNr", size : sizeof(RestCfg::gasDevMdbNr) }, //
				{ ctype : cpxINT, id:5, ofs: offsetof(RestCfg, dustDevMdbNr), Name : "DustDevMdbNr", size : sizeof(RestCfg::dustDevMdbNr) }, //
				{ ctype : cpxINT, id:6, ofs: offsetof(RestCfg, gasFiltrType), Name : "GasFiltrType", size : sizeof(RestCfg::gasFiltrType) }, //
				{ ctype : cpxINT, id:7, ofs: offsetof(RestCfg, filtrFIRLength), Name : "GasFiltrFIRLength", size : sizeof(RestCfg::filtrFIRLength) }, //
				{ ctype : cpxFLOAT, id:8, ofs: offsetof(RestCfg, filtrIRConst), Name : "GasFiltrIRConst", size : sizeof(RestCfg::filtrIRConst) }, //
#if(SENSOR_NOISE)
				{ ctype : cpxINT, id:9, ofs: offsetof(RestCfg, noiseFiltrType), Name : "NoiseFiltrType", size : sizeof(RestCfg::noiseFiltrType) }, //
				{ ctype : cpxINT, id:10, ofs: offsetof(RestCfg, noiseFiltrFIRLength), Name : "NoiseFiltrFIRLength", size : sizeof(RestCfg::noiseFiltrFIRLength) }, //
				{ ctype : cpxFLOAT, id:11, ofs: offsetof(RestCfg, noiseFiltrIRConst), Name : "NoiseFiltrIRConst", size : sizeof(RestCfg::noiseFiltrIRConst) }, //
#endif
				{ ctype : cpxNULL } };

const CpxChildInfo restCfgGroup = { //
		itemCnt: 1, //
				itemSize : sizeof(RestCfg), //
				defs: RestCfgDescr, //
		};

const CpxDescr ConfigDscr[] = { //
		//dev
				{ ctype : cpxCHILD, id:1, ofs: offsetof(CfgRec, R.dev), Name : "dev", 1, exPtr :&devInfoGroup }, //
				//tcp
#if (ETHERNET)
				{ ctype : cpxCHILD, id:2, ofs: offsetof(CfgRec, R.tcp), Name : "tcp", 1, exPtr :&tcpGroup }, //
#endif
				//bg
				{ ctype : cpxCHILD, id:3, ofs: offsetof(CfgRec, R.bg96), Name : "bg", 1, exPtr :&bgGroup }, //
				//mqtt
				{ ctype : cpxCHILD, id:4, ofs: offsetof(CfgRec, R.mqtt), Name : "mqtt", 1, exPtr :&mqttGroup }, //
				//rest
				{ ctype : cpxCHILD, id:5, ofs: offsetof(CfgRec, R.rest), Name : "ex", 1, exPtr :&restCfgGroup }, //
				//sensors
				{ ctype : cpxCHILD, id:6, ofs: offsetof(CfgRec, R.sensExist), Name : "tab", 1, exPtr :&sensorOnOffGroup }, //
#if (HEATER)
				{ ctype : cpxCHILD, id:7, ofs: offsetof(CfgRec, R.heater), Name : "heater", 1, exPtr :&heaterGroup }, //
#endif
#if (LED_MATRIX)
				{ ctype : cpxCHILD, id:8, ofs: offsetof(CfgRec, R.ledMatrix), Name : "led_matrix", 1, exPtr :&ledMatrixGroup }, //
#endif
				{ ctype : cpxNULL }

		};

CxString *Config::jsonbuf;

Config::Config() {
	jsonbuf = NULL;

}

HAL_StatusTypeDef Config::LoadIntern(uint32_t adr) {
	memcpy(&data, (void*) adr, sizeof(data));

	if (data.Sign != CFG_SIGN || data.size != sizeof(data))
		return HAL_DATA_ERR;

	if (!MdbCrc::Check(data.tab_b, sizeof(data)))
		return HAL_CRC_ERR;

	return HAL_OK;
}

void Config::prepareToSave() {
	data.Sign = CFG_SIGN;
	data.size = sizeof(data);
	data.ver = 1;
	data.R.Sign = CFG_SIGN_U;
	data.R.size = CFG_REC_SIZE_USER;
	MdbCrc::Set(data.tab_b, sizeof(data) - 2);
}

bool Config::saveRtc() {
	if (!Korekt()) {
		prepareToSave();
		HAL_PWR_EnableBkUpAccess();
		memcpy((void*) CFG_ADDR_RTCRAM, data.tab_b, sizeof(data));
		HAL_PWR_DisableBkUpAccess();
		return true;
	} else {
		return false;
	}
}

#if 0

int8_t write_to_backup_sram( uint8_t *data, uint16_t bytes, uint16_t offset ) {
  const uint16_t backup_size = 0x1000;
  uint8_t* base_addr = (uint8_t *) BKPSRAM_BASE;
  uint16_t i;
  if( bytes + offset >= backup_size ) {
    /* ERROR : the last byte is outside the backup SRAM region */
    return -1;
  }
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_BKPSRAM, ENABLE);
  /* disable backup domain write protection */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);   // set RCC->APB1ENR.pwren
  PWR_BackupAccessCmd(ENABLE);                          // set PWR->CR.dbp = 1;
  /** enable the backup regulator (used to maintain the backup SRAM content in
    * standby and Vbat modes).  NOTE : this bit is not reset when the device
    * wakes up from standby, system reset or power reset. You can check that
    * the backup regulator is ready on PWR->CSR.brr, see rm p144 */
  PWR_BackupRegulatorCmd(ENABLE);     // set PWR->CSR.bre = 1;
  for( i = 0; i < bytes; i++ ) {
    *(base_addr + offset + i) = *(data + i);
  }
  PWR_BackupAccessCmd(DISABLE);                     // reset PWR->CR.dbp = 0;
  return 0;
}
#endif

HAL_StatusTypeDef Config::saveFlash() {
	HAL_StatusTypeDef st = HAL_OK;
	if (saveRtc()) {
		st = HAL_FLASH_Unlock();
		if (st == HAL_OK) {

			HAL_StatusTypeDef st = HAL_FLASH_Unlock();
			if (st == HAL_OK) {
				FLASH_EraseInitTypeDef EraseRec;
				uint32_t SectorError;

				EraseRec.TypeErase = FLASH_TYPEERASE_SECTORS;
				EraseRec.Sector = FLASH_SECTOR_11;
				EraseRec.VoltageRange = FLASH_VOLTAGE_RANGE_3;
				EraseRec.NbSectors = 1;
				EraseRec.Banks = FLASH_BANK_1;

				HAL_IWDG_Refresh(&hiwdg);
				st = HAL_FLASHEx_Erase(&EraseRec, &SectorError);  //coś WatchDog się wpieprza
				if (st != HAL_OK) {
					st = HAL_FLASHEx_Erase(&EraseRec, &SectorError);
				}

				if (st == HAL_OK) {
					dword Adr = CFG_ADDR_FLASH;
					for (int i = 0; i < CFG_REC_SIZE_4; i++) {
						dword d2 = data.tab_32[i];
						st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Adr, d2);
						if (st != HAL_OK) {
							break;
						}
						Adr += 4;
					}
				}
				HAL_FLASH_Lock();
			}
		}
		if (st == HAL_OK) {
			if (memcmp(data.tab_b, (void*) CFG_ADDR_FLASH, sizeof(data)) != 0)
				st = HAL_CMP_ERR;
		}
	} else {
		st = HAL_CFG_KOREKTED;
	}
	return st;

}

void Config::Zero() {
	memset(&data, 0, sizeof(data));
	saveRtc();
}

bool Config::checkRange(float v, float min, float max) {
	if (v < min) {
		return true;
	}
	if (v > max) {
		return true;
	}
	return false;
}

bool Config::checkRange(int v, int min, int max) {
	if (v < min) {
		return true;
	}
	if (v > max) {
		return true;
	}
	return false;
}

bool Config::Korekt() {
	bool r = false;

	if (data.R.rest.gasFiltrType < 0 || data.R.rest.gasFiltrType >= 3) {
		data.R.rest.gasFiltrType = 0;
		r = true;
	}

	if (checkRange(data.R.rest.filtrFIRLength, 0, 120)) {
		data.R.rest.filtrFIRLength = 120;
		r = true;
	}

	if (checkRange(data.R.rest.filtrIRConst, 0.0, 1.0)) {
		data.R.rest.filtrIRConst = 0.05;
		r = true;
	}

#if(SENSOR_NOISE)
	if (data.R.rest.noiseFiltrType < 0 || data.R.rest.noiseFiltrType >= 3) {
		data.R.rest.noiseFiltrType = 0;
		r = true;
	}

	if (checkRange(data.R.rest.noiseFiltrFIRLength, 0, 120)) {
		data.R.rest.filtrFIRLength = 120;
		r = true;
	}

	if (checkRange(data.R.rest.noiseFiltrIRConst, 0.0, 1.0)) {
		data.R.rest.filtrIRConst = 0.05;
		r = true;
	}
#endif

#if (HEATER)
	if (checkRange(data.R.heater.humidityON, 0.0, 100.0)) {
		data.R.heater.humidityON = 75;
		r = true;
	}
	if (checkRange(data.R.heater.humidityOFF, 0.0, 100.0)) {
		data.R.heater.humidityOFF = 50;
		r = true;
	}

	if (data.R.heater.humidityON <= data.R.heater.humidityOFF) {
		data.R.heater.humidityON = 75;
		data.R.heater.humidityOFF = 50;
		r = true;
	}

	if (checkRange(data.R.heater.tempON, -30.0, 50.0)) {
		data.R.heater.tempON = 2;
		r = true;
	}
	if (checkRange(data.R.heater.tempOFF, -30.0, 50.0)) {
		data.R.heater.tempOFF = 2;
		r = true;
	}

	if (data.R.heater.tempON >= data.R.heater.tempOFF) {
		data.R.heater.tempON = 2;
		data.R.heater.tempOFF = 10;
		r = true;
	}
#endif

	if (data.R.rest.gasDevMdbNr == 0) {
		data.R.rest.gasDevMdbNr = 73;
		r = true;
	}
	if (data.R.rest.dustDevMdbNr == 0) {
		data.R.rest.dustDevMdbNr = 73;
		r = true;
	}

#if (DEV_DUST_INT_EXT)
	if (data.R.dev.dustInpType != dust_Intern && data.R.dev.dustInpType != dust_Extern) {
		data.R.dev.dustInpType = dust_Intern;
		r = true;
	}
#endif

#if (DEV_DUST_INTERN_TYP)
	if (data.R.dev.dustSensorIntType >= dustT_Cnt) {
		data.R.dev.dustSensorIntType = dustT_SPS30;
	}
#endif

	if (data.R.sensExist[ssUNKNOWN]) {
		data.R.sensExist[ssUNKNOWN] = 0;
		r = true;
	}

	return r;
}

void Config::Default() {
	memset(&data, 0, sizeof(data));
	strcpy(data.R.dev.SerialNr, "W00001");
	data.R.dev.timeZoneShift = 1;
#if (DEV_DUST_INTERN_TYP)
	data.R.dev.dustSensorIntType = dustT_SPS30;
#endif
#if (DEV_DUST_INT_EXT)
	data.R.dev.dustInpType = dust_Intern;
#endif
	data.R.dev.gpsLatitude = 0;
	data.R.dev.gpsLongitude = 0;

	strcpy(data.R.bg96.SimPin, "");
	strcpy(data.R.bg96.ApnName, "playmetric");
	data.R.bg96.autoStart = true;
	data.R.bg96.rssiRefreshTime = 60;
	data.R.bg96.gps.refreshTime = 30;
	data.R.bg96.gps.Mode = 1;
	strcpy(data.R.bg96.BgEcho, "0000");  //1114
	strcpy(data.R.bg96.ntp.SvrName, "pool.ntp.org");
	data.R.bg96.ntp.WaitTime = 5000;
	data.R.bg96.ntp.RefreshTime = 600;

	//mqtt
	data.R.mqtt.autoOpenMqttSvr = true;
	data.R.mqtt.mqttSendInterval = 120;  //2 minuty
	strcpy(data.R.mqtt.SvrName, "iot-endpoint.syngeos.pl");
	strcpy(data.R.mqtt.userName, "iot");
	strcpy(data.R.mqtt.password, "sensorI0t");
	strcpy(data.R.mqtt.varNamePub, "iot/sensor");
	data.R.mqtt.useSSL = false;
	data.R.mqtt.usePSK = false;
	data.R.mqtt.maxLoginTime = 10;
	data.R.mqtt.keepAlive = 0;
	data.R.mqtt.SvrPortNoSSL = 1884;
	data.R.mqtt.SvrPortSSL = 8883;
	data.R.mqtt.SvrPortPSK = 8885;
	data.R.mqtt.retain = 1;
	data.R.mqtt.qos = 1;

	data.R.rest.gasDevMdbNr = 73;
	data.R.rest.dustDevMdbNr = 73;
	data.R.rest.mdb1dbgLevel = 0;
	data.R.rest.mdb2dbgLevel = 0;
	data.R.rest.mdb3dbgLevel = 0;
	data.R.rest.gasFiltrType = 0;
	data.R.rest.filtrFIRLength = 120;
	data.R.rest.filtrIRConst = 0.05;
#if (HEATER)
	data.R.heater.useNTCtemp = true;
	data.R.heater.runExternal = 0;
	data.R.heater.runInternal = 1;
	data.R.heater.showMsg = 0;
	data.R.heater.tempON = 10;
	data.R.heater.tempOFF = 20;
	data.R.heater.humidityON = 99;
	data.R.heater.humidityOFF = 50;
	data.R.heater.humidityEnab = false;
#endif
	for (int i = 1; i < SENSOR_CNT; i++)
		data.R.sensExist[i] = 1;
#if (LED_MATRIX)
	data.R.ledMatrix.limitTab[0] = 13;
	data.R.ledMatrix.limitTab[1] = 35;
	data.R.ledMatrix.limitTab[2] = 55;
	data.R.ledMatrix.limitTab[3] = 75;
	data.R.ledMatrix.limitTab[4] = 110;
	data.R.ledMatrix.run = true;
	data.R.ledMatrix.autoSend = true;
	data.R.ledMatrix.histereza = 2;
#endif
	saveRtc();
}

HAL_StatusTypeDef Config::Init(OutStream *strm) {
	HAL_StatusTypeDef st_r = LoadIntern(CFG_ADDR_RTCRAM);
	HAL_StatusTypeDef st_f = LoadIntern(CFG_ADDR_FLASH);

	if ((st_r == HAL_OK) && (st_f != HAL_OK)) {
		strm->oMsgX(colWHITE, "Copy_cfg RTC->FLASH");
		saveFlash();
	}
	if ((st_f == HAL_OK) && (st_r != HAL_OK)) {
		strm->oMsgX(colWHITE, "Copy_cfg FLASH->RTC");
		saveRtc();
	}

	if (st_r == HAL_OK) {
		LoadIntern(CFG_ADDR_RTCRAM);
		strm->oMsgX(colWHITE, "Cfg from RTC");
		if (Korekt()) {
			strm->oMsgX(colRED, "Corection of CFG");
		}

		return HAL_OK;
	}
	if (st_f == HAL_OK) {
		LoadIntern(CFG_ADDR_FLASH);
		strm->oMsgX(colWHITE, "Cfg from FLASH");
		if (Korekt()) {
			strm->oMsgX(colRED, "Corection of CFG");
		}
		return HAL_OK;
	}
	strm->oMsgX(colRED, "Cfg default");
	Default();
	return HAL_ERROR;
}

void Config::funListUni(OutStream *strm, const char *cmd, void *arg, bool withK) {
	Config *cfg = (Config*) arg;
	char tok[20];

	Cpx cpx;
	cpx.init(ConfigDscr, &cfg->data);

	if (Token::get(&cmd, tok, sizeof(tok))) {
		if (strcmp(tok, "rtc") == 0) {
			cpx.init(ConfigDscr, (void*) CFG_ADDR_RTCRAM);
			strm->oMsgX(colYELLOW, "RTC");
		} else if (strcmp(tok, "flash") == 0) {
			cpx.init(ConfigDscr, (void*) CFG_ADDR_FLASH);
			strm->oMsgX(colYELLOW, "FLASH");
		}
	}
	if (!withK)
		cpx.list(strm);
	else
		cpx.listk(strm);
}

void Config::funList(OutStream *strm, const char *cmd, void *arg) {
	funListUni(strm, cmd, arg, false);
}

void Config::funListK(OutStream *strm, const char *cmd, void *arg) {
	funListUni(strm, cmd, arg, true);
}

void Config::funSet(OutStream *strm, const char *cmd, void *arg) {
	Config *cfg = (Config*) arg;
	char tok[30];

	if (Token::get(&cmd, tok, sizeof(tok))) {
		char valB[60];
		valB[0] = 0;
		Token::get(&cmd, valB, sizeof(valB));
		Cpx cpx;
		cpx.init(ConfigDscr, &cfg->data);
		if (cpx.set(tok, valB)) {
			strm->oMsgX(colGREEN, "[%s]=(%s) OK", tok, valB);
		} else {
			strm->oMsgX(colRED, "[%s]=(%s) Error", tok, valB);
		}
	}
}

void Config::funDefault(OutStream *strm, const char *cmd, void *arg) {
	Config *cfg = (Config*) arg;
	strm->oMsgX(colWHITE, "Ustawienia domyślne");
	cfg->Default();
}

void Config::funSave(OutStream *strm, const char *cmd, void *arg) {
	Config *cfg = (Config*) arg;
	if (cfg->saveRtc())
		strm->oMsgX(colWHITE, "Saved to RTCRam");
	else
		strm->oMsgX(colRED, "Cfg Corrected");
}

void Config::funSaveFlash(OutStream *strm, const char *cmd, void *arg) {
	Config *cfg = (Config*) arg;
	HAL_StatusTypeDef st = cfg->saveFlash();
	strm->oMsgX(HAL_getColor(st), "Save to Flash & RTCRam : %s", HAL_getErrStr(st));
}

void Config::funInit(OutStream *strm, const char *cmd, void *arg) {
	Config *cfg = (Config*) arg;
	HAL_StatusTypeDef st = cfg->LoadIntern(CFG_ADDR_RTCRAM);
	strm->oMsgX(HAL_getColor(st), "Load from RTCRam : %s", HAL_getErrStr(st));
}

void Config::funInitFlash(OutStream *strm, const char *cmd, void *arg) {
	Config *cfg = (Config*) arg;
	HAL_StatusTypeDef st = cfg->LoadIntern(CFG_ADDR_FLASH);
	strm->oMsgX(HAL_getColor(st), "Load from Flash: %s", HAL_getErrStr(st));
}

void Config::funShowDef(OutStream *strm, const char *cmd, void *arg) {
	Config *cfg = (Config*) arg;

	Cpx cpx;
	cpx.init(ConfigDscr, &cfg->data);
	cpx.showDef(strm);
}

void Config::funShowInfo(OutStream *strm, const char *cmd, void *arg) {
	Config *cfg = (Config*) arg;

	Cpx cpx;
	cpx.init(ConfigDscr, &cfg->data);
	cpx.info(strm);
}

void Config::funShowJson(OutStream *strm, const char *cmd, void *arg) {
	Config *cfg = (Config*) arg;

	if (jsonbuf == NULL) {
		jsonbuf = new CxString(JSON_SIZE);
	}
	if (jsonbuf != NULL) {
		jsonbuf->clear();

		Cpx cpx;
		cpx.init(ConfigDscr, &cfg->data);

		cpx.buildjson(jsonbuf);
		strm->oMsgX(colWHITE, "Len=%d", jsonbuf->len());
		strm->oBufX(colYELLOW, jsonbuf->p(), jsonbuf->len());
	}
}

void Config::funHelp(OutStream *strm, const char *cmd, void *arg) {
	if (strm->oOpen(colWHITE)) {
		strm->oMsg("Cfg Help");
		strm->oMsg("--------------------");
		strm->oMsg("DustSensorType: 0-SPS30(Sensirion), 1-HPMA(Honeywel), 2-PMSA003, 3-PMS5003ST");
		strm->oMsg("GasFiltrType: 0-OFF, 1-FIR, 2-IR");
		strm->oClose();
	}

}

const ShellItemFx menuCfgFx[] = { //
		{ "list", "[F8] pokaż ustawienia, parametr [ |rtc|flash]", Config::funList }, //
				{ "listk", "pokaż ustawienia, parametr [ |rtc|flash]", Config::funListK }, //
				{ "set", "ustaw wartość", Config::funSet }, //
				{ "default", "wartości domyślne", Config::funDefault }, //
				{ "save", "save to Rtc RAM", Config::funSave }, //
				{ "saveflash", "save to Flash", Config::funSaveFlash }, //
				{ "init", "reload cfg from RtcRAM", Config::funInit }, //
				{ "initflash", "reload cfg from Flash", Config::funInitFlash }, //
				{ "json", "pokaż definicje", Config::funShowJson }, //
				{ "def", "pokaż definicje", Config::funShowDef }, //
				{ "info", "pokaż informacje", Config::funShowInfo }, //
				{ "help", "znaczenie nirktórych nastaw", Config::funHelp }, //
				{ NULL, NULL } };

void Config::shell(OutStream *strm, const char *cmd) {
	execMenuCmd(strm, menuCfgFx, cmd, this, "Config Menu");
}

