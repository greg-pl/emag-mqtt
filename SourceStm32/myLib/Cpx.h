/*
 * Cpx.h
 *
 *  Created on: Dec 11, 2020
 *      Author: Grzegorz
 */

#ifndef CPX_H_
#define CPX_H_

#include "stdint.h"
#include "ErrorDef.h"

#include <IOStream.h>
#include <CxString.h>
#include <CxBuf.h>

typedef enum {
	cpxNULL, //
	cpxCHILD, //
	cpxSTR, //
	cpxQUOTASTR, //
	cpxBOOL, //
	cpxBYTE, //
	cpxWORD, //
	cpxHEXWORD, //
	cpxINT, //
	cpxFLOAT, //
	cpxTIME, //
	cpxSELECT, //
	cpxSELECT_BYTE, //
	cpxIP, // todo Add staff
} CpxType;

typedef struct {
	CpxType ctype;
	int id;
	uint32_t ofs;
	const char *Name;
	int size; // Wartość 0 odpowiada wartości 1
	const void *exPtr;
} CpxDescr;

#define flagSHOWBR   0x0001  //przed listowaniem pokaz linie rozdzielająca
typedef struct {
	int itemCnt;
	int itemSize;
	const CpxDescr *defs;
	uint32_t flags;
} CpxChildInfo;

typedef struct {
	CpxDescr descr;
	void *data;
} FndRes;

typedef struct {
	int val;
	const char *cap;
} CpxSelectItem;

typedef struct {
	int min; // wartość min
	int max; // wartość max
} CpxIntDefItem;

typedef const char* (*FunCpxFloatGetFrm)(int idx);

typedef struct {
	float min; // wartość min
	float max; // wartość max
	FunCpxFloatGetFrm getFrm;  //pobranie stringu formatującego
} CpxFloatDefItem;


class IdHist {
	bool mAddNum;
	int mDeep;
	uint8_t *mem;
	int mPtr;
	int mIdx;
public:
	IdHist(bool addNum, int deep);
	~IdHist();
	void add(int id);
	void delLast();
	void buildStr(CxString *cstr);
	void fillHistBin(CxBuf *cxBuf);

	void loadfromStr(const char *txt);
	void loadfromMem(const uint8_t *dt);

	int getIDX() {
		return mem[mIdx];
	}
	bool isLastIdx() {
		return (mIdx == mPtr - 1);
	}
	void next();
	void prev();
	bool isAdd() {
		return mAddNum;
	}
};

class Cpx {
private:
	enum {
		WBUF_SIZE = 160,
	};
	const CpxDescr *mDef;
	void *mData;

	static bool find(CxString *token, Cpx *result, const CpxDescr *aDef, void *aData, const char *name);
	static bool find(Cpx *result, const CpxDescr *aDef, void *aData, IdHist *idHist);
	static bool set(const Cpx *itemToSet, const char *txt);
	static bool setBin(const Cpx *itemToSet, const uint8_t *val, uint8_t dt_sz);
	static void getAsTxt(const CpxDescr *def, const void *data, char *buf, int max);

	bool find(Cpx *result, const char *name);
	bool find(Cpx *result, IdHist *idHist);

	void showDef(OutStream *strm, const CpxDescr *def, char *wBuf, char *space);
	void info(OutStream *strm, CxString *wstr, CxString *front, IdHist *keyHist, int idx);
	void list(OutStream *strm, CxString *wstr, CxString *front, IdHist *keyHist, int idx, int frontSize);
	void buildjson(CxString *out, char *space, int idx, uint8_t flags);
	void buildBinCfg(CxBuf *cxBuf, IdHist *idHist);
	static bool getSelectVal(const void *exPtr, const char *txt, int *ret);
	static bool getSelectTxt(char *txt, int max, const void *exPtr, int val);
	static void showSelectItems(CxString *wstr, const void *exPtr);
	static bool getOgrInt(const void *exPtr,const char *txt,int *val);
	static bool getOgrFloat(const void *exPtr, const char *txt, float *val);

public:
	static const char *getSelectTxtPtr(const void *exPtr, int val);

public:
	void init(const CpxDescr *aDef, const void *aData);
	bool next();
	bool isEof();
	const char *getDscrName();

	bool setItem(const char *val);
	bool set(const char *name, const char *val);
	bool set(const char *name, const char *val, int *fndItemId);

	bool setk(const char *idStr, const char *val);
	bool setm(const uint8_t *idMem, const uint8_t *val, uint8_t dt_sz);

	void info(OutStream *strm);
	void list(OutStream *strm);
	void listk(OutStream *strm);
	void showDef(OutStream *strm);
	void buildjson(CxString *outStr);
	void getAsTxt(char *buf, int max);
	void getAsTxt(CxString *cstr);
	void buildBinCfg(CxBuf *cxBuf);
	TStatus InsertChanges(OutStream *strm, const uint8_t *data, int len);
	static void BuildKeyStr(char *txt, int max, const uint8_t *key);

public:
	static int atoi_hex(const char *txt);
	static bool CheckFloatItem(float *pf, const CpxFloatDefItem *def, float defau);
	static bool CheckIntItem(int *pf, const CpxIntDefItem *def, int defau);
	static bool CheckByteItem(uint8_t *pf, const CpxIntDefItem *def, uint8_t defau);

};

#endif /* CPX_H_ */
