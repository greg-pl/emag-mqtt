/*
 * ErrorDef.h
 *
 *  Created on: 22 kwi 2021
 *      Author: Grzegorz
 */

#ifndef TRACKOBJ_ERRORDEF_H_
#define TRACKOBJ_ERRORDEF_H_

//definicja statusów
typedef enum {
	stOK = 0,    // HAL_OK
  	stError =1,  // HAL_ERROR    = 0x01U,
  	stBusy=2,    // HAL_BUSY     = 0x02U,
  	stTimeOut=3, // HAL_TIMEOUT  = 0x03U

	stCrcError,   //
	stCompareErr, //
	stNoSemafor,  //
	stDataErr,    //
	stMdbErr1,    // bład 1 modbus'a
	stMdbErr2,    // bład 2 modbus'a
	stMdbErr3,    // bład 3 modbus'a
	stMdbErr4,    // bład 4 modbus'a
	stMdbError,   //
	stCfgDataErr, //
	stFlashWriteError, //


	stNotAllignedData, // dane nie wyrownane do rozmiaru uint32
	stAdrTooBig, 	// zbyt daleki adres
	stLengthNoAllign, 	// długość nie jest wielokrotnościa 4
	stCompareError, //bład porównania
	stNotClear, 	//39 - flash nie jest czysty
	stTooBigBuffer, //40 - flash nie jest czysty


	stERR_NO_START_BYTE=50, //
	stERR_SUM, //
	stFRAME_TOO_BIG, //


	stLAST, // musi być przecinek

}TStatus;

#endif /* TRACKOBJ_ERRORDEF_H_ */
