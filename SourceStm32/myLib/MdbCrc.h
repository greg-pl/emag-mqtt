/*
 * MdbCrc.h
 *
 *  Created on: Dec 5, 2020
 *      Author: Grzegorz
 */

#ifndef MDBCRC_H_
#define MDBCRC_H_

#include "stdint.h"

class MdbCrc {
private:
	static uint16_t Proceed(uint16_t Crc, uint8_t inp);
public:
	static void Set(uint8_t *p, int cnt);
	static uint16_t Build(const uint8_t *p, int cnt);
	static bool Check(const uint8_t *p, int cnt);
};

#endif /* MDBCRC_H_ */
