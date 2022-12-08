/*
 * Cpx.cpp
 *
 *  Created on: Dec 11, 2020
 *      Author: Grzegorz
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include <Cpx.h>
#include <utils.h>

void Cpx::init(const CpxDef *aDef, const void *aData) {
	mDef = aDef;
	mData = (void*) (int) aData;
}

const CpxDef* Cpx::find(const char *name) {
	const CpxDef *def = mDef;
	while (def->ctype != cpxNULL) {
		if (strcmp(def->Name, name) == 0) {
			return def;
		}
		def++;
	}
	return NULL;
}

bool Cpx::set(const char *name, const char *txt) {
	const CpxDef *def = find(name);
	if (def == NULL)
		return false;
	return set(def, mData, txt);
}

int Cpx::atoi_hex(const char *txt) {
	if (txt[0] == '0' && (txt[1] == 'x' || txt[1] == 'X')) {
		return strtol(txt, NULL, 16);
	} else {
		return atoi(txt);
	}

}

bool Cpx::set(const CpxDef *def, void *data, const char *txt) {

	void *ptr = (void*) ((uint32_t) data + def->ofs);

	switch (def->ctype) {
	case cpxNULL:
		return false;
	case cpxBREAK_LINE:
		break;
	case cpxSTR: {
		char *dst = (char*) ptr;
		strncpy(dst, txt, def->size);
		dst[def->size - 1] = 0;
	}
	case cpxQUOTASTR: {
		char *dst = (char*) ptr;
		Token::copyNoQuota(dst, def->size, txt);
	}
		break;
	case cpxBOOL: {
		bool *pb = (bool*) ptr;
		*pb = atoi(txt);
	}
		break;

	case cpxBYTE: {
		uint8_t *pb = (uint8_t*) ptr;
		*pb = atoi_hex(txt);
	}
		break;
	case cpxHEXWORD:
	case cpxWORD: {
		uint16_t *pb = (uint16_t*) ptr;
		*pb = atoi_hex(txt);
	}
		break;

	case cpxINT: {
		int *pi = (int*) ptr;
		*pi = atoi_hex(txt);
	}
		break;
	case cpxFLOAT: {
		float *pf = (float*) ptr;
		*pf = atof(txt);
	}
		break;

	case cpxIP: {
		ip4_addr_t *pf = (ip4_addr_t*) ptr;
		return ipaddr_aton(txt, pf);
	}
		break;
	case cpxTIME: {
		TDATE *tm = (TDATE*) ptr;
		const char *ptr = txt;
		TimeTools::parseDate(&ptr, tm);
		TimeTools::parseTime(&ptr, tm);
	}
		break;
	}
	return true;
}

const char *const ctypeNameTab[] = { "NULL", "TAB", "BREAK_LINE", "STR", "QUOTASTR", "BOOL", "BYTE", "WORD", "HEXWORD", "INT", "FLOAT", "IP" };

const char* getCodeName(CpxType typ) {
	int N = sizeof(ctypeNameTab) / sizeof(ctypeNameTab[0]);
	if (typ < N)
		return ctypeNameTab[typ];
	else
		return "<?>";
}

void Cpx::showDef(OutStream *strm, const CpxDef *def, char *wBuf, char *space) {
	while (def->ctype != cpxNULL) {
		const GroupInfo *grInfo = NULL;
		int n = snprintf(wBuf, WBUF_SIZE, "%styp=%s ofs=%u name=[%s] size=%u", space, getCodeName(def->ctype), (int) def->ofs, def->Name, def->size);

		switch (def->ctype) {
		case cpxFLOAT:
			if (def->exPtr != NULL) {
				FunCpxFloatGetFrm fun = (FunCpxFloatGetFrm) (def->exPtr);
				const char *frm = fun(0);
				n += snprintf(&wBuf[n], WBUF_SIZE - n, " Frm=[%s]", frm);
			}
			break;
		case cpxTAB:
			grInfo = (const GroupInfo*) def->exPtr;
			n += snprintf(&wBuf[n], WBUF_SIZE - n, " N=%u sz=%u", grInfo->itemCnt, grInfo->itemSize);
			break;
		default:
			break;
		}
		strm->oWr(wBuf);
		if (def->ctype == cpxTAB) {
			int m = strlen(space);
			space[m + 0] = ' ';
			space[m + 1] = ' ';
			space[m + 2] = 0;
			showDef(strm, grInfo->defs, wBuf, space);
			space[m] = 0;
		}
		def++;
	}

}

void Cpx::showDef(OutStream *strm) {
	if (strm->oOpen(colWHITE)) {
		char spaceStr[40];
		char wbuf[WBUF_SIZE];
		spaceStr[0] = 0;
		showDef(strm, mDef, wbuf, spaceStr);
		strm->oClose();
	}
}

void Cpx::getAsTxt(const CpxDef *def, const void *data, char *buf, int max) {
	void *ptr = (void*) ((uint32_t) data + def->ofs);

	switch (def->ctype) {
	case cpxSTR:
		snprintf(buf, max, (char*) ptr);
		break;
	case cpxQUOTASTR:
		Token::setQuotaStr(buf, (const char*) ptr, max);
		break;

	case cpxBOOL:
		snprintf(buf, max, "%u", *(bool*) ptr);
		break;
	case cpxBYTE:
		snprintf(buf, max, "%u", *(uint8_t*) ptr);
		break;
	case cpxHEXWORD:
		snprintf(buf, max, "0X%04X", *(uint16_t*) ptr);
		break;
	case cpxWORD:
		snprintf(buf, max, "%u", *(uint16_t*) ptr);
		break;
	case cpxINT:
		snprintf(buf, max, "%u", *(int*) ptr);
		break;
	case cpxFLOAT: {
		const char *frm = "%f";
		if (def->exPtr != NULL) {
			FunCpxFloatGetFrm fun = (FunCpxFloatGetFrm) (def->exPtr);
			frm = fun(0);
		}
		snprintf(buf, max, frm, (double) (*(float*) ptr));
	}
		break;

	case cpxIP:
		ipaddr_ntoa_r((ip4_addr_t* ) ptr, buf, max);
		break;
	case cpxTIME: {
		const TDATE *tm = (const TDATE*) ptr;
		char tmBuf[20];
		TimeTools::DtTmStr(tmBuf, tm);
		snprintf(buf, max, tmBuf);
	}
		break;

	default:
		break;
	}

}

void Cpx::list(OutStream *strm, char *space, int idx) {

	const CpxDef *def = mDef;
	while (def->ctype != cpxNULL) {

		void *ptr = (void*) ((uint32_t) mData + def->ofs);

		switch (def->ctype) {
		case cpxBREAK_LINE:
			strm->oMsg("--- %s ----------------------", def->Name);
			break;
		case cpxSTR:
			strm->oMsg("%s%s:%s", space, def->Name, (char*) ptr);
			break;
		case cpxQUOTASTR: {
			char buf[100];
			Token::setQuotaStr(buf, (const char*) ptr, sizeof(buf));
			strm->oMsg("%s%s:%s", space, def->Name, buf);

		}
			break;

		case cpxBOOL: {
			bool *pb = (bool*) ptr;
			strm->oMsg("%s%s:%u", space, def->Name, (int) (*pb));
		}
			break;
		case cpxBYTE: {
			uint8_t *pb = (uint8_t*) ptr;
			strm->oMsg("%s%s:%u", space, def->Name, *pb);
		}
			break;
		case cpxWORD: {
			uint16_t *pb = (uint16_t*) ptr;
			strm->oMsg("%s%s:%u", space, def->Name, *pb);
		}
			break;
		case cpxHEXWORD: {
			uint16_t *pb = (uint16_t*) ptr;
			strm->oMsg("%s%s:0X%04X", space, def->Name, *pb);
		}
			break;
		case cpxINT: {
			int *pi = (int*) ptr;
			strm->oMsg("%s%s:%d", space, def->Name, *pi);
		}
			break;
		case cpxFLOAT: {
			const char *frm = "%f";
			if (def->exPtr != NULL) {
				FunCpxFloatGetFrm fun = (FunCpxFloatGetFrm) (def->exPtr);
				frm = fun(idx);
			}
			float *pf = (float*) ptr;
			char buf[20];
			snprintf(buf, sizeof(buf), frm, *pf);
			strm->oMsg("%s%s:%s", space, def->Name, buf);
		}
			break;

		case cpxIP: {
			ip4_addr_t *pf = (ip4_addr_t*) ptr;
			char txt[20];
			ipaddr_ntoa_r(pf, txt, sizeof(txt));
			strm->oMsg("%s%s:%s", space, def->Name, txt);
		}
			break;
		case cpxTIME: {
			const TDATE *tm = (const TDATE*) ptr;
			char tmBuf[20];
			TimeTools::DtTmStr(tmBuf, tm);
			strm->oMsg("%s%s:%s", space, def->Name, tmBuf);
		}
			break;

		case cpxTAB: {
			const GroupInfo *grInfo = (const GroupInfo*) def->exPtr;
			strm->oMsg("%s%s:TAB[%u]", space, def->Name, grInfo->itemCnt);
			int m = strlen(space);
			uint8_t *dt = (uint8_t*) ptr;
			for (int k = 0; k < grInfo->itemCnt; k++) {
				strm->oMsg("%s%u.", space, k);
				Cpx child;
				child.init(grInfo->defs, dt);
				strcat(space, "  ");
				child.list(strm, space, k);
				space[m] = 0;
				dt += grInfo->itemSize;
			}
		}
			break;

		default:
			break;
		}
		def++;
	}
}

void Cpx::list(OutStream *strm) {
	if (strm->oOpen(colWHITE)) {
		char spaceStr[40];
		spaceStr[0] = 0;
		list(strm, spaceStr, 0);
		strm->oClose();
	}
}

#define JSON_NL "\r\n"
#define JSON_SPACE1 "    "

int Cpx::buildjson(char *mbuf, int max, char *space, int idx, bool lastItem) {
	const CpxDef *def = mDef;
	int n = snprintf(mbuf, max, "%s{" JSON_NL, space);
	int spM = strlen(space);
	strcat(space, JSON_SPACE1);
	bool firstItem = true;
	while (def->ctype != cpxNULL) {
		void *ptr = (void*) ((uint32_t) mData + def->ofs);
		if (max - n < 1)
			return -1;

		bool addItem = true;
		switch (def->ctype) {
		case cpxFLOAT: {
			float *pf = (float*) ptr;
			float f = *pf;
			if (isnanf(f))
				addItem = false;
		}
			break;
		default:
			break;
		}

		if (addItem) {
			if (firstItem) {
				firstItem = false;
			} else {
				n += snprintf(&mbuf[n], max - n, "," JSON_NL);
				if (max - n < 1)
					return -1;
			}

			n += snprintf(&mbuf[n], max - n, "%s\"%s\": ", space, def->Name);
			if (max - n < 1)
				return -1;

			switch (def->ctype) {
			case cpxBREAK_LINE:
				break;
			case cpxQUOTASTR:
			case cpxSTR:
				n += snprintf(&mbuf[n], max - n, "\"%s\"", (char*) ptr);
				break;

			case cpxBOOL: {
				bool *pb = (bool*) ptr;
				n += snprintf(&mbuf[n], max - n, "%s", FalseTrue(*pb));
			}
				break;
			case cpxBYTE: {
				uint8_t *pb = (uint8_t*) ptr;
				n += snprintf(&mbuf[n], max - n, "%u", *pb);
			}
				break;
			case cpxWORD: {
				uint16_t *pb = (uint16_t*) ptr;
				n += snprintf(&mbuf[n], max - n, "%u", *pb);
			}
				break;
			case cpxHEXWORD: {
				uint16_t *pb = (uint16_t*) ptr;
				n += snprintf(&mbuf[n], max - n, "0X%04X", *pb);
			}
				break;
			case cpxINT: {
				int *pi = (int*) ptr;
				n += snprintf(&mbuf[n], max - n, "%u", *pi);
			}
				break;
			case cpxFLOAT: {
				const char *frm = "%f";
				if (def->exPtr != NULL) {
					FunCpxFloatGetFrm fun = (FunCpxFloatGetFrm) (def->exPtr);
					frm = fun(idx);
				}
				float *pf = (float*) ptr;
				n += snprintf(&mbuf[n], max - n, frm, *pf);
			}
				break;

			case cpxIP: {
				ip4_addr_t *pf = (ip4_addr_t*) ptr;
				char txt[20];
				ipaddr_ntoa_r(pf, txt, sizeof(txt));
				n += snprintf(&mbuf[n], max - n, txt);
			}
				break;
			case cpxTIME: {
				const TDATE *tm = (const TDATE*) ptr;
				char tmBuf[20];
				TimeTools::DtTmStr(tmBuf, tm);
				n += snprintf(&mbuf[n], max - n,"\"%s\"", tmBuf);
			}
				break;

			case cpxTAB: {
				n += snprintf(&mbuf[n], max - n, "[" JSON_NL);
				if (max - n < 5)
					return -1;

				uint8_t *dt = (uint8_t*) ptr;
				const GroupInfo *grInfo = (const GroupInfo*) def->exPtr;

				int spMM = strlen(space);
				strcat(space, JSON_SPACE1);

				for (int ii = 0; ii < grInfo->itemCnt; ii++) {
					Cpx child;
					child.init(grInfo->defs, dt);
					int k = child.buildjson(&mbuf[n], max - n, space, ii, (ii == grInfo->itemCnt - 1));
					if (k < 0)
						return k;
					n += k;
					dt += grInfo->itemSize;
				}
				space[spMM] = 0;
				n += snprintf(&mbuf[n], max - n, "%s]", space);
				if (max - n < 2)
					return -1;
			}
				break;

			default:
				break;
			}
		}
		def++;
	}
	n += snprintf(&mbuf[n], max - n, JSON_NL);
	if (max - n < 1)
		return -1;

	if (n < max) {
		space[spM] = 0;
		if (lastItem)
			n += snprintf(&mbuf[n], max - n, "%s}" JSON_NL, space);
		else
			n += snprintf(&mbuf[n], max - n, "%s}," JSON_NL, space);
	}
	return n;
}

int Cpx::buildjson(char *buf, int max) {
	char spaceStr[40];
	spaceStr[0] = 0;
	return buildjson(buf, max, spaceStr, 0, true);
}
