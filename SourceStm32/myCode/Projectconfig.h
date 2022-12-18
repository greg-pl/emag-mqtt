#pragma once

#define ETHERNET				0
#define LED_MATRIX 				0
#define SSD1306 				0
#define HEATER   				0
#define TEMP_NTC	        	0

#define DEV_BMP338				1
#define DEV_SHT35				1
#define DEV_S873				1
#define DEV_AIR_DET_RS			0
#define DEV_NOISE       		0

#define DEV_DUST_PMSA   	    0
#define DEV_DUST_PMS5003ST  	0  // with Formaldehyde sensor
#define DEV_DUST_SPS30    		0
#define DEV_DUST_MDB     		0  //dust sensor connected as modbus device

#define DEV_DUST_INTERN_CNT  	(DEV_DUST_PMSA + DEV_DUST_SPS30 + DEV_DUST_PMS5003ST)
#define DEV_DUST_INTERN 		(DEV_DUST_INTERN_CNT>0)
#define DEV_DUST_INTERN_TYP 	(DEV_DUST_INTERN_CNT>1)  			// more then one sensor type
#define DEV_DUST      			(DEV_DUST_INTERN || DEV_DUST_MDB)
#define DEV_DUST_INT_EXT		(DEV_DUST_INTERN && DEV_DUST_MDB)   // sensor can be working as internal or external

#define MDB1_EXIST				(DEV_NOISE)
#define MDB2_EXIST				(DEV_S873 || DEV_AIR_DET_RS)
#define MDB3_EXIST				(DEV_DUST_MDB)
#define MDB_EXIST				(MDB1_EXIST || MDB2_EXIST ||MDB3_EXIST)

#define SENSOR_TEMPERATURE      (DEV_SHT35 || DEV_BMP338 || DEV_S873 || DEV_AIR_DET_RS)
#define SENSOR_HUMIDITY	        (DEV_SHT35 || DEV_S873)
#define SENSOR_PRESSURE	        (DEV_BMP338 || DEV_S873)
#define SENSOR_DUST        		(DEV_DUST)
#define SENSOR_CH_SO       		(DEV_DUST_PMS5003ST)						// sensor only in DEV_DUST_PMS5003ST
#define SENSOR_NOISE       		(DEV_NOISE)
#define SENSOR_SO2              (DEV_AIR_DET_RS)
#define SENSOR_NO2              (DEV_AIR_DET_RS)
#define SENSOR_O3               (DEV_AIR_DET_RS)
#define SENSOR_CO               (DEV_AIR_DET_RS || DEV_S873)
#define SENSOR_CO2              (DEV_S873)


#define FORCE_S873				1  // mesurements from S873 more important
