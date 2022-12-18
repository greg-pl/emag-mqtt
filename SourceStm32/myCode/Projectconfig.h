
#pragma once


#define ETHERNET				0
#define LED_MATRIX 				0
#define SSD1306 				0
#define HEATER   				0
#define TEMP_NTC	        	0

#define DEV_NOISE       		0

#define DEV_DUST_PMSA   	    1
#define DEV_DUST_PMS5003ST  	1  // z czujnikiem Formaldehyde
#define DEV_DUST_SPS30    		1
#define DEV_DUST_MDB     		1  //dust sensor connected as modbus device

#define DEV_DUST_INTERN_CNT  	(DEV_DUST_PMSA + DEV_DUST_SPS30 + DEV_DUST_PMS5003ST)
#define DEV_DUST_INTERN 		(DEV_DUST_INTERN_CNT>0)
#define DEV_DUST_INTERN_TYP 	(DEV_DUST_INTERN_CNT>1)  // więcej niż jeden obsługiwany typ czujnika
#define DEV_DUST      			(DEV_DUST_INTERN || DEV_DUST_MDB)
#define DEV_DUST_INT_EXT		(DEV_DUST_INTERN && DEV_DUST_MDB)    // czujnik mozliwy do podłączenia wewnętrzne i przez modbus

#define SENSOR_CH_SO       		(DEV_DUST_PMS5003ST)						// czujnik dostępny tylko w DEV_DUST_PMS5003ST
#define SENSOR_DUST        		(DEV_DUST)
#define SENSOR_NOISE       		(DEV_NOISE)
