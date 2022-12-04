/*
 * Cpx.h
 *
 *  Created on: Dec 11, 2020
 *      Author: Grzegorz
 */

#ifndef CPX_H_
#define CPX_H_

#include <IOStream.h>
#include "stdint.h"

#include "cmsis_os.h"
#include "lwip.h"


typedef enum {
	cpxNULL, cpxTAB, cpxBREAK_LINE, cpxSTR, cpxQUOTASTR, cpxBOOL, cpxBYTE, cpxWORD, cpxHEXWORD, cpxINT, cpxFLOAT, cpxIP,cpxTIME,
} CpxType;

typedef struct {
	CpxType ctype;
	uint32_t ofs;
	const char *Name;
	int size; // Wartość 0 odpowiada wartości 1
	const void *exPtr;
} CpxDef;

typedef struct {
	int itemCnt;
	int itemSize;
	const CpxDef *defs;
} GroupInfo;

typedef const char* (*FunCpxFloatGetFrm)(int idx);

class Cpx {
private:
	enum {
		WBUF_SIZE = 160,
	};
	const CpxDef *mDef;
	void *mData;
	const CpxDef* find(const char *name);
	void showDef(OutStream *strm, const CpxDef *def, char *wBuf, char *space);
	void list(OutStream *strm, char *space, int idx);
	int buildjson(char *buf, int max, char *space, int idx, bool lastItem);
public:
	void init(const CpxDef *aDef, const void *aData);
	bool set(const char *name, const char *val);
	void list(OutStream *strm);
	void showDef(OutStream *strm);
	static bool set(const CpxDef *def, void *data, const char *txt);
	int buildjson(char *buf, int max);
public:
	static void getAsTxt(const CpxDef *def, const void *data, char *buf, int max);
	static int atoi_hex(const char *txt);

};

#endif /* CPX_H_ */
