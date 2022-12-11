/*
 * DustSensorDef.h
 *
 *  Created on: 31 gru 2020
 *      Author: Grzegorz
 */

#ifndef DUSTSENSORBASE_H_
#define DUSTSENSORBASE_H_

#include <IOStream.h>
#include "stm32f4xx_hal.h"
#include "utils.h"

typedef struct {
	float pm1_0;
	float pm2_5;
	float pm10;
	float Formaldehyde; // tylko dla PMS5003ST
} DustMeasRec;

class DustSensorBase {
private:
	osMutexId mDustMutex;
protected:
	enum {
		MAX_MEAS_VALID = 5000, //maksymalny czas między paczkami danych
	};
	bool openMutex(int tm);
	void closeMutex();
	struct {
		uint32_t mLastRdDataTick;
		DtFilter filterPM1_0;
		DtFilter filterPM2_5;
		DtFilter filterPM10;
		DtFilter filterFormaldehyde;
	} exportDt;
public:
	virtual void StartMeas()=0;
	virtual void StopMeas()=0;
	virtual void shell(OutStream *strm, const char *cmd)=0;
	virtual void setPower(bool on)=0;
	virtual HAL_StatusTypeDef Init(SignaledClass *signObj)=0;
	virtual void tick()=0;
	virtual bool isError();
public:
	DustSensorBase();
	HAL_StatusTypeDef getMeas(DustMeasRec *meas);
};

class DustSensorNull: public DustSensorBase {
public:
	virtual void StartMeas();
	virtual void StopMeas();
	virtual void shell(OutStream *strm, const char *cmd);
	virtual HAL_StatusTypeDef getMeas(DustMeasRec *meas);
	virtual void setPower(bool on);
	virtual HAL_StatusTypeDef Init(SignaledClass *signObj);
	virtual void tick();
};

#endif /* DUSTSENSORBASE_H_ */
