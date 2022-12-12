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

extern "C" const TcpInterfDef* getTcpDef() {
	static TcpInterfDef def;
	memcpy(&def, &config->data.R.tcp, sizeof(def));
	return &def;
}

#define  CFG_SIGN  0x2367A3B7
#define  CFG_SIGN_P  0x2367A3B2
#define  CFG_SIGN_U  0x2367A3BB

#define CFG_ADDR_RTCRAM   (BKPSRAM_BASE+0)
#define CFG_ADDR_FLASH    0x080E0000

extern "C" const char* getGpsFormat(int idx);

const CpxDescr ConfigDscr[] = { //
		{ ctype : cpxSTR, ofs: offsetof(CfgRec, P.SerialNr), Name : "SerialNr", sizeof(CfgRec::P.SerialNr) }, //
				{ ctype : cpxSTR, ofs: offsetof(CfgRec, R.DevInfo), Name : "DevInfo", sizeof(CfgRec::R.DevInfo) }, //
				{ ctype : cpxINT, ofs: offsetof(CfgRec, R.timeZoneShift), Name : "TimeZoneShift", size : sizeof(CfgRec::R.timeZoneShift) }, //
				{ ctype : cpxFLOAT, ofs: offsetof(CfgRec, R.gpsLatitude), Name : "GpsLatitude", size : sizeof(CfgRec::R.gpsLatitude), exPtr :(const void*) getGpsFormat }, //
				{ ctype : cpxFLOAT, ofs: offsetof(CfgRec, R.gpsLongitude), Name : "GpsLongitude", size : sizeof(CfgRec::R.gpsLongitude), exPtr :(const void*) getGpsFormat }, //

				//{ ctype : cpxBREAK_LINE, ofs: 0, Name : "Ethernet", 0 }, //
				{ ctype : cpxBYTE, ofs: offsetof(CfgRec, R.tcp.dhcp), Name : "tcp_dhcp", size: sizeof(CfgRec::R.tcp.dhcp) }, //
				{ ctype : cpxIP, ofs: offsetof(CfgRec, R.tcp.ip), Name : "tcp_ip", size: sizeof(CfgRec::R.tcp.ip) }, //
				{ ctype : cpxIP, ofs: offsetof(CfgRec, R.tcp.mask), Name : "tcp_mask", size: sizeof(CfgRec::R.tcp.mask) }, //
				{ ctype : cpxIP, ofs: offsetof(CfgRec, R.tcp.gw), Name : "tcp_gw", size: sizeof(CfgRec::R.tcp.gw) }, //
				{ ctype : cpxIP, ofs: offsetof(CfgRec, R.tcp.dns1), Name : "tcp_dns1", size: sizeof(CfgRec::R.tcp.dns1) }, //
				{ ctype : cpxIP, ofs: offsetof(CfgRec, R.tcp.dns2), Name : "tcp_dns2", size: sizeof(CfgRec::R.tcp.dns2) }, //

				//{ ctype : cpxBREAK_LINE, ofs: 0, Name : "BG96", 0 }, //
				{ ctype : cpxSTR, ofs: offsetof(CfgRec, R.bg96.BgEcho), Name : "BgEcho", size: sizeof(CfgRec::R.bg96.BgEcho) }, //
				{ ctype : cpxSTR, ofs: offsetof(CfgRec, R.bg96.SimPin), Name : "SimPin", size: sizeof(CfgRec::R.bg96.SimPin) }, //
				{ ctype : cpxINT, ofs: offsetof(CfgRec, R.bg96.rssiRefreshTime), Name : "RssiRefreshTime", size : sizeof(CfgRec::R.bg96.rssiRefreshTime) }, //
				{ ctype : cpxINT, ofs: offsetof(CfgRec, R.bg96.gps.Mode), Name : "GpsMode", size: sizeof(CfgRec::R.bg96.gps.Mode) }, //
				{ ctype : cpxINT, ofs: offsetof(CfgRec, R.bg96.gps.refreshTime), Name : "GpsRefreshTime", size : sizeof(CfgRec::R.bg96.gps.refreshTime) }, //
				{ ctype : cpxBOOL, ofs: offsetof(CfgRec, R.bg96.gps.setRtcTime), Name : "GpsSetRtcTime", size : sizeof(CfgRec::R.bg96.gps.setRtcTime) }, //

				{ ctype : cpxSTR, ofs: offsetof(CfgRec, R.bg96.ApnName), Name : "ApnName", size: sizeof(CfgRec::R.bg96.ApnName) }, //
				{ ctype : cpxSTR, ofs: offsetof(CfgRec, R.bg96.ntp.SvrName), Name : "NtpSvrName", size: sizeof(CfgRec::R.bg96.ntp.SvrName) }, //
				{ ctype : cpxINT, ofs: offsetof(CfgRec, R.bg96.ntp.WaitTime), Name : "NtpWaitTime", size : sizeof(CfgRec::R.bg96.ntp.WaitTime) }, //
				{ ctype : cpxINT, ofs: offsetof(CfgRec, R.bg96.ntp.RefreshTime), Name : "NtpRefreshTime", size : sizeof(CfgRec::R.bg96.ntp.RefreshTime) }, //

				{ ctype : cpxBOOL, ofs: offsetof(CfgRec, R.bg96.autoStart), Name : "AutoStart", size: sizeof(CfgRec::R.bg96.autoStart) }, //
				{ ctype : cpxBOOL, ofs: offsetof(CfgRec, R.bg96.autoOpenMqttSvr), Name : "AutoOpenMqttSvr", size : sizeof(CfgRec::R.bg96.autoOpenMqttSvr) }, //
				{ ctype : cpxINT, ofs: offsetof(CfgRec, R.bg96.mqttSendInterval), Name : "MqttSendInterval", size : sizeof(CfgRec::R.bg96.mqttSendInterval) }, //

				//{ ctype : cpxBREAK_LINE, ofs: 0, Name : "MQTT", 0 }, //
				{ ctype : cpxSTR, ofs: offsetof(CfgRec, R.mqtt.SvrName), Name : "MqttSvrName", size: sizeof(CfgRec::R.mqtt.SvrName) }, //
				{ ctype : cpxWORD, ofs: offsetof(CfgRec, R.mqtt.SvrPortNoSSL), Name : "MqttPortNoSSL", size : sizeof(CfgRec::R.mqtt.SvrPortNoSSL) }, //
				{ ctype : cpxWORD, ofs: offsetof(CfgRec, R.mqtt.SvrPortSSL), Name : "MqttPortSSL", size: sizeof(CfgRec::R.mqtt.SvrPortSSL) }, //
				{ ctype : cpxWORD, ofs: offsetof(CfgRec, R.mqtt.SvrPortPSK), Name : "MqttPortPSK", size: sizeof(CfgRec::R.mqtt.SvrPortPSK) }, //
				{ ctype : cpxSTR, ofs: offsetof(CfgRec, R.mqtt.userName), Name : "MqttUserName", size: sizeof(CfgRec::R.mqtt.userName) }, //
				{ ctype : cpxSTR, ofs: offsetof(CfgRec, R.mqtt.password), Name : "MqttUserPassword", size : sizeof(CfgRec::R.mqtt.password) }, //
				{ ctype : cpxBOOL, ofs: offsetof(CfgRec, R.mqtt.useSSL), Name : "MqttUseSSL", size: sizeof(CfgRec::R.mqtt.useSSL) }, //
				{ ctype : cpxBOOL, ofs: offsetof(CfgRec, R.mqtt.usePSK), Name : "MqttUsePSK", size: sizeof(CfgRec::R.mqtt.usePSK) }, //
				{ ctype : cpxINT, ofs: offsetof(CfgRec, R.mqtt.maxLoginTime), Name : "MqttMaxLoginTime", size : sizeof(CfgRec::R.mqtt.maxLoginTime) }, //
				{ ctype : cpxINT, ofs: offsetof(CfgRec, R.mqtt.keepAlive), Name : "MqttKeepAlive", size : sizeof(CfgRec::R.mqtt.keepAlive) }, //
				{ ctype : cpxINT, ofs: offsetof(CfgRec, R.mqtt.qos), Name : "MqttQOS", size: sizeof(CfgRec::R.mqtt.qos) }, //
				{ ctype : cpxBOOL, ofs: offsetof(CfgRec, R.mqtt.retain), Name : "MqttRetain", size: sizeof(CfgRec::R.mqtt.retain) }, //
				{ ctype : cpxSTR, ofs: offsetof(CfgRec, R.mqtt.varNamePub), Name : "MqttSendVarName", size : sizeof(CfgRec::R.mqtt.varNamePub) }, //
				{ ctype : cpxSTR, ofs: offsetof(CfgRec, R.mqtt.varNamePub2), Name : "MqttSendVar2Name", size : sizeof(CfgRec::R.mqtt.varNamePub2) }, //

				//{ ctype : cpxBREAK_LINE, ofs: 0, Name : "Inne", 0 }, //

				{ ctype : cpxBYTE, ofs: offsetof(CfgRec, P.dustInpType), Name : "DustInpType" }, //
				{ ctype : cpxBYTE, ofs: offsetof(CfgRec, P.dustSensorType), Name : "DustSensorType" }, //
				{ ctype : cpxBOOL, ofs: offsetof(CfgRec, R.rest.ledOff), Name : "PcbLedOff", size: sizeof(CfgRec::R.rest.ledOff) }, //
				{ ctype : cpxINT, ofs: offsetof(CfgRec, R.rest.mdb1dbgLevel), Name : "Mdb1DbgLevel", size : sizeof(CfgRec::R.rest.mdb1dbgLevel) }, //
				{ ctype : cpxINT, ofs: offsetof(CfgRec, R.rest.mdb2dbgLevel), Name : "Mdb2DbgLevel", size : sizeof(CfgRec::R.rest.mdb2dbgLevel) }, //
				{ ctype : cpxINT, ofs: offsetof(CfgRec, R.rest.mdb3dbgLevel), Name : "Mdb3DbgLevel", size : sizeof(CfgRec::R.rest.mdb3dbgLevel) }, //
				{ ctype : cpxINT, ofs: offsetof(CfgRec, R.rest.gasDevMdbNr), Name : "GasDevMdbNr", size : sizeof(CfgRec::R.rest.gasDevMdbNr) }, //
				{ ctype : cpxINT, ofs: offsetof(CfgRec, R.rest.dustDevMdbNr), Name : "DustDevMdbNr", size : sizeof(CfgRec::R.rest.dustDevMdbNr) }, //

				//{ ctype : cpxBREAK_LINE, ofs: 0, Name : "Sensors", 0 }, //
				{ ctype : cpxBOOL, ofs: offsetof(CfgRec, R.exDev.sensExist[ssTEMPERATURE]), Name : "existSensTemper", size : sizeof(bool) }, //
				{ ctype : cpxBOOL, ofs: offsetof(CfgRec, R.exDev.sensExist[ssHUMIDITY]), Name : "existSensHumidity", size: sizeof(bool) }, //
				{ ctype : cpxBOOL, ofs: offsetof(CfgRec, R.exDev.sensExist[ssPRESSURE]), Name : "existSensPressure", size: sizeof(bool) }, //
				{ ctype : cpxBOOL, ofs: offsetof(CfgRec, R.exDev.sensExist[ssPM1_0]), Name : "existSensPM1_0", size: sizeof(bool) }, //
				{ ctype : cpxBOOL, ofs: offsetof(CfgRec, R.exDev.sensExist[ssPM2_5]), Name : "existSensPM2_5", size: sizeof(bool) }, //
				{ ctype : cpxBOOL, ofs: offsetof(CfgRec, R.exDev.sensExist[ssPM10]), Name : "existSensPM10", size : sizeof(bool) }, //
				{ ctype : cpxBOOL, ofs: offsetof(CfgRec, R.exDev.sensExist[ssNO2]), Name : "existSensNO2", size: sizeof(bool) }, //
				{ ctype : cpxBOOL, ofs: offsetof(CfgRec, R.exDev.sensExist[ssO3]), Name : "existSensO3", size: sizeof(bool) }, //
				{ ctype : cpxBOOL, ofs: offsetof(CfgRec, R.exDev.sensExist[ssCO]), Name : "existSensCO", size: sizeof(bool) }, //
				{ ctype : cpxBOOL, ofs: offsetof(CfgRec, R.exDev.sensExist[ssCO2]), Name : "existSensCO2", size: sizeof(bool) }, //
				{ ctype : cpxBOOL, ofs: offsetof(CfgRec, R.exDev.sensExist[ssSO2]), Name : "existSensSO2", size: sizeof(bool) }, //
				{ ctype : cpxBOOL, ofs: offsetof(CfgRec, R.exDev.sensExist[ssCh2o]), Name : "existSensCH2O", size: sizeof(bool) }, //
				{ ctype : cpxBOOL, ofs: offsetof(CfgRec, R.exDev.sensExist[ssNOISE]), Name : "existSensNoise", size: sizeof(bool) }, //

				{ ctype : cpxINT, ofs: offsetof(CfgRec, R.exDev.gasFiltrType), Name : "GasFiltrType", size : sizeof(CfgRec::R.exDev.gasFiltrType) }, //
				{ ctype : cpxINT, ofs: offsetof(CfgRec, R.exDev.filtrFIRLength), Name : "GasFiltrFIRLength", size : sizeof(CfgRec::R.exDev.filtrFIRLength) }, //
				{ ctype : cpxFLOAT, ofs: offsetof(CfgRec, R.exDev.filtrIRConst), Name : "GasFiltrIRConst", size : sizeof(CfgRec::R.exDev.filtrIRConst) }, //

				{ ctype : cpxINT, ofs: offsetof(CfgRec, R.exDev.noiseFiltrType), Name : "NoiseFiltrType", size : sizeof(CfgRec::R.exDev.noiseFiltrType) }, //
				{ ctype : cpxINT, ofs: offsetof(CfgRec, R.exDev.noiseFiltrFIRLength), Name : "NoiseFiltrFIRLength", size : sizeof(CfgRec::R.exDev.noiseFiltrFIRLength) }, //
				{ ctype : cpxFLOAT, ofs: offsetof(CfgRec, R.exDev.noiseFiltrIRConst), Name : "NoiseFiltrIRConst", size : sizeof(CfgRec::R.exDev.noiseFiltrIRConst) }, //

				//{ ctype : cpxBREAK_LINE, ofs: 0, Name : "Heater", 0 }, //
				{ ctype : cpxBOOL, ofs: offsetof(CfgRec, R.exDev.heater.useNTCtemp), Name : "heater_useNtcTemp", size: sizeof(bool) }, //
				{ ctype : cpxBOOL, ofs: offsetof(CfgRec, R.exDev.heater.runExternal), Name : "heater_runExternal", size: sizeof(bool) }, //
				{ ctype : cpxBOOL, ofs: offsetof(CfgRec, R.exDev.heater.runInternal), Name : "heater_runInternal", size: sizeof(bool) }, //
				{ ctype : cpxBYTE, ofs: offsetof(CfgRec, R.exDev.heater.showMsg), Name : "heater_showMsg", size: sizeof(bool) }, //

				{ ctype : cpxFLOAT, ofs: offsetof(CfgRec, R.exDev.heater.tempON), Name : "heater_tempON", size : sizeof(CfgRec::R.exDev.heater.tempON) }, //
				{ ctype : cpxFLOAT, ofs: offsetof(CfgRec, R.exDev.heater.tempOFF), Name : "heater_tempOFF", size : sizeof(CfgRec::R.exDev.heater.tempOFF) }, //
				{ ctype : cpxFLOAT, ofs: offsetof(CfgRec, R.exDev.heater.humidityON), Name : "heater_humidityON", size : sizeof(CfgRec::R.exDev.heater.humidityON) }, //
				{ ctype : cpxFLOAT, ofs: offsetof(CfgRec, R.exDev.heater.humidityOFF), Name : "heater_humidityOFF", size : sizeof(CfgRec::R.exDev.heater.humidityOFF) }, //
				{ ctype : cpxBOOL, ofs: offsetof(CfgRec, R.exDev.heater2.humidityEnab), Name : "heater_humidityENAB", size : sizeof(CfgRec::R.exDev.heater2.humidityEnab) }, //

				//{ ctype : cpxBREAK_LINE, ofs: 0, Name : "LedMatrix", 0 }, //

				{ ctype : cpxBOOL, ofs: offsetof(CfgRec, R.rest.ledMatrixRun), Name : "ledMatrix_run", size : sizeof(CfgRec::R.rest.ledMatrixRun) }, //
				{ ctype : cpxBOOL, ofs: offsetof(CfgRec, R.rest.faceAutoSend), Name : "ledMatrix_auto", size : sizeof(CfgRec::R.rest.faceAutoSend) }, //
				{ ctype : cpxINT, ofs: offsetof(CfgRec, R.rest.faceLevel), Name : "ledMatrix_level", size : sizeof(CfgRec::R.rest.faceLevel) }, //

				{ ctype : cpxFLOAT, ofs: offsetof(CfgRec, R.rest.faceHistereza), Name : "ledMatrix_histereza", size : sizeof(CfgRec::R.rest.faceHistereza) }, //
				{ ctype : cpxFLOAT, ofs: offsetof(CfgRec, R.rest.faceLimitTab[0]), Name : "ledMatrix_lev1", size : sizeof(CfgRec::R.rest.faceLimitTab[0]) }, //
				{ ctype : cpxFLOAT, ofs: offsetof(CfgRec, R.rest.faceLimitTab[1]), Name : "ledMatrix_lev2", size : sizeof(CfgRec::R.rest.faceLimitTab[1]) }, //
				{ ctype : cpxFLOAT, ofs: offsetof(CfgRec, R.rest.faceLimitTab[2]), Name : "ledMatrix_lev3", size : sizeof(CfgRec::R.rest.faceLimitTab[2]) }, //
				{ ctype : cpxFLOAT, ofs: offsetof(CfgRec, R.rest.faceLimitTab[3]), Name : "ledMatrix_lev4", size : sizeof(CfgRec::R.rest.faceLimitTab[3]) }, //
				{ ctype : cpxFLOAT, ofs: offsetof(CfgRec, R.rest.faceLimitTab[4]), Name : "ledMatrix_lev5", size : sizeof(CfgRec::R.rest.faceLimitTab[4]) }, //

				{ ctype : cpxNULL }

		};

Config::Config() {

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
	data.P.Sign = CFG_SIGN_P;
	data.P.size = CFG_REC_SIZE_PRODUCER;
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

	if (data.R.exDev.gasFiltrType < 0 || data.R.exDev.gasFiltrType >= 3) {
		data.R.exDev.gasFiltrType = 0;
		r = true;
	}

	if (checkRange(data.R.exDev.filtrFIRLength, 0, 120)) {
		data.R.exDev.filtrFIRLength = 120;
		r = true;
	}

	if (checkRange(data.R.exDev.filtrIRConst, 0.0, 1.0)) {
		data.R.exDev.filtrIRConst = 0.05;
		r = true;
	}

	//NOISE
	if (data.R.exDev.noiseFiltrType < 0 || data.R.exDev.noiseFiltrType >= 3) {
		data.R.exDev.noiseFiltrType = 0;
		r = true;
	}

	if (checkRange(data.R.exDev.noiseFiltrFIRLength, 0, 120)) {
		data.R.exDev.filtrFIRLength = 120;
		r = true;
	}

	if (checkRange(data.R.exDev.noiseFiltrIRConst, 0.0, 1.0)) {
		data.R.exDev.filtrIRConst = 0.05;
		r = true;
	}

	if (checkRange(data.R.exDev.heater.humidityON, 0.0, 100.0)) {
		data.R.exDev.heater.humidityON = 75;
		r = true;
	}
	if (checkRange(data.R.exDev.heater.humidityOFF, 0.0, 100.0)) {
		data.R.exDev.heater.humidityOFF = 50;
		r = true;
	}

	if (data.R.exDev.heater.humidityON <= data.R.exDev.heater.humidityOFF) {
		data.R.exDev.heater.humidityON = 75;
		data.R.exDev.heater.humidityOFF = 50;
		r = true;
	}

	if (checkRange(data.R.exDev.heater.tempON, -30.0, 50.0)) {
		data.R.exDev.heater.tempON = 2;
		r = true;
	}
	if (checkRange(data.R.exDev.heater.tempOFF, -30.0, 50.0)) {
		data.R.exDev.heater.tempOFF = 2;
		r = true;
	}

	if (data.R.exDev.heater.tempON >= data.R.exDev.heater.tempOFF) {
		data.R.exDev.heater.tempON = 2;
		data.R.exDev.heater.tempOFF = 10;
		r = true;
	}

	if (data.R.rest.gasDevMdbNr == 0) {
		data.R.rest.gasDevMdbNr = 73;
		r = true;
	}
	if (data.R.rest.dustDevMdbNr == 0) {
		data.R.rest.dustDevMdbNr = 73;
		r = true;
	}

	if (data.P.dustInpType != dust_Intern && data.P.dustInpType != dust_Extern) {
		data.P.dustInpType = dust_Intern;
		r = true;
	}

	if (data.R.exDev.sensExist[ssUNKNOWN]) {
		data.R.exDev.sensExist[ssUNKNOWN] = 0;
		r = true;
	}

	return r;
}

void Config::Default() {
	memset(&data, 0, sizeof(data));
	strcpy(data.P.SerialNr, "W00001");
	data.R.timeZoneShift = 1;
	data.P.dustSensorType = dustT_SPS30;
	data.P.dustInpType = dust_Intern;

	strcpy(data.R.bg96.SimPin, "");
	strcpy(data.R.bg96.ApnName, "playmetric");
	data.R.bg96.autoStart = true;
	data.R.bg96.autoOpenMqttSvr = true;
	data.R.bg96.mqttSendInterval = 120;  //2 minuty
	data.R.bg96.rssiRefreshTime = 60;
	data.R.bg96.gps.refreshTime = 30;
	data.R.bg96.gps.Mode = 1;
	strcpy(data.R.bg96.BgEcho, "1114");

	strcpy(data.R.bg96.ntp.SvrName, "pool.ntp.org");
	data.R.bg96.ntp.WaitTime = 5000;
	data.R.bg96.ntp.RefreshTime = 600;

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

	data.R.gpsLatitude = 0;
	data.R.gpsLongitude = 0;
	data.R.rest.gasDevMdbNr = 73;
	data.R.rest.dustDevMdbNr = 73;
	data.R.rest.mdb1dbgLevel = 0;
	data.R.rest.mdb2dbgLevel = 0;
	data.R.rest.mdb3dbgLevel = 0;

	data.R.exDev.gasFiltrType = 0;
	data.R.exDev.filtrFIRLength = 120;
	data.R.exDev.filtrIRConst = 0.05;

	data.R.exDev.heater.useNTCtemp = true;
	data.R.exDev.heater.runExternal = 0;
	data.R.exDev.heater.runInternal = 1;
	data.R.exDev.heater.showMsg = 0;
	data.R.exDev.heater.tempON = 10;
	data.R.exDev.heater.tempOFF = 20;
	data.R.exDev.heater.humidityON = 99;
	data.R.exDev.heater.humidityOFF = 50;
	data.R.exDev.heater2.humidityEnab = false;

	data.R.exDev.sensExist[ssTEMPERATURE] = 1;
	data.R.exDev.sensExist[ssHUMIDITY] = 1;
	data.R.exDev.sensExist[ssPRESSURE] = 1;
	data.R.exDev.sensExist[ssPM1_0] = 1;
	data.R.exDev.sensExist[ssPM2_5] = 1;
	data.R.exDev.sensExist[ssPM10] = 1;
	data.R.exDev.sensExist[ssNO2] = 0;
	data.R.exDev.sensExist[ssO3] = 0;

	data.R.rest.faceLimitTab[0] = 13;
	data.R.rest.faceLimitTab[1] = 35;
	data.R.rest.faceLimitTab[2] = 55;
	data.R.rest.faceLimitTab[3] = 75;
	data.R.rest.faceLimitTab[4] = 110;

	data.R.rest.ledMatrixRun = true;
	data.R.rest.faceAutoSend = true;
	data.R.rest.faceHistereza = 2;

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

/*


 }

 */
void Config::funList(OutStream *strm, const char *cmd, void *arg) {
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
	cpx.list(strm);
}

void Config::funSet(OutStream *strm, const char *cmd, void *arg) {
	Config *cfg = (Config*) arg;
	char tok[20];

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
		{ "list", "pokaż ustawienia, parametr [ |rtc|flash]", Config::funList }, //
				{ "set", "ustaw wartość", Config::funSet }, //
				{ "default", "wartości domyślne", Config::funDefault }, //
				{ "save", "save to Rtc RAM", Config::funSave }, //
				{ "saveflash", "save to Flash", Config::funSaveFlash }, //
				{ "init", "reload cfg from RtcRAM", Config::funInit }, //
				{ "initflash", "reload cfg from Flash", Config::funInitFlash }, //
				{ "def", "pokaż definicje", Config::funShowDef }, //
				{ "help", "znaczenie nirktórych nastaw", Config::funHelp }, //
				{ NULL, NULL } };

void Config::shell(OutStream *strm, const char *cmd) {
	execMenuCmd(strm, menuCfgFx, cmd, this, "Config Menu");
}

