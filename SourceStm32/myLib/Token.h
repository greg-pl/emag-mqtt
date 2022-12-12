
#ifndef TOKEN_H_
#define TOKEN_H_

#include "_ansi.h"
#include "stdint.h"
#include "myDef.h"


class Token {
private:
	static bool isSep(const char *sep, char ch);
public:
	static void trim(const char **inp);
	static bool get(const char **inp, const char *sepStr, char *tok, int size, char *pSepBf, char *pSepAf);
	static bool get(const char **inp, const char *sepStr, char *tok, int size, char *pSepBf);
	static bool get(const char **inp, char *tok, int size, char *pSepBf);
	static bool get(const char **inp, char *tok, int size);
	static bool get(const char **inp);

	static bool getAsInt(const char **inp, int *val);
	static bool getAsInt(const char **inp, const char *sepStr, int *val);
	static bool getAsBool(const char **inp, bool *val);
	static bool getAsDate(const char **inp, TDATE *tm);


	static bool getAsFloat(const char **inp, float *val);
	static bool chgCtrlChar(char *dst, const char *src, int max);
	static void remooveEOL(char *line);
	static bool remooveQuota(char *line);
	static void copyNoQuota(char *dst, int max, const char *src);
	static void setQuotaStr(char *dst, const char *txt, int max);
	static void shiftLeft(char *buf, int dist);
	static int parseInt(const char *ptr, int len);
};


#endif
