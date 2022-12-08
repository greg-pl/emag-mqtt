/*
 * myDef.h
 *
 *  Created on: 25 lis 2019
 *      Author: Grzegorz
 */

#ifndef INC_MYDEF_H_
#define INC_MYDEF_H_

#include "stdint.h"
#include "lwip.h"

#define  HAL_CRC_ERR ((HAL_StatusTypeDef) 0x04)
#define  HAL_CMP_ERR ((HAL_StatusTypeDef) 0x05)
#define  HAL_NOWR_EXEC ((HAL_StatusTypeDef) 0x06)
#define  HAL_NO_VALUE  ((HAL_StatusTypeDef) 0x07)
#define  HAL_NO_SEMF   ((HAL_StatusTypeDef) 0x08)
#define  HAL_DATA_ERR  ((HAL_StatusTypeDef) 0x09)
#define  HAL_SUM_ERR  ((HAL_StatusTypeDef) 0x010)
#define  HAL_CFG_KOREKTED ((HAL_StatusTypeDef) 0x011)

#define SEC_LABEL   __attribute__ ((section (".label")))
#define SEC_NOINIT  __attribute__ ((section (".noinit")))



typedef unsigned char byte;
typedef unsigned int  dword;
typedef unsigned short word;

#define false 0
#define true 1

enum{
	tmSrcUNKNOWN=0,
	tmSrcMANUAL, //wprowadzony ręcznie
	tmSrcNTP, //protokół NTP
	tmSrcGPS, //GPS
	tmFirmVer,
};

typedef struct {
	uint8_t rk; //
	uint8_t ms; //
	uint8_t dz; //
	uint8_t gd; //
	uint8_t mn; //
	uint8_t sc; //
	uint8_t se; // setne części sekundy
	uint8_t timeSource; //
} TDATE;

typedef struct {
	word ver;
	word rev;
	TDATE time;
} VerInfo;


typedef enum{
	measRDY, // dane gotowe
	measWAIT, // stan oczekiwania
	measWAIT_LONG, // stan d�ugiego czekania
	measUNAV, //dane niedost�pne
} MeasState;


typedef struct {
		uint8_t dhcp;
		ip4_addr_t ip;
		ip4_addr_t mask;
		ip4_addr_t gw;
		ip4_addr_t dns1;
		ip4_addr_t dns2;
} TcpInterfDef;


#endif /* INC_MYDEF_H_ */
