/*
 * Token.cpp
 *
 *  Created on: 25 mar 2021
 *      Author: Grzegorz
 */

// Token
//-------------------------------------------------------------------------------------
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include "Token.h"
#include "utils.h"

bool Token::isSep(const char *sep, char ch) {
	return (strchr(sep, ch) != NULL);
}

void Token::trim(const char **inp) {
	const char *ptr = *inp;

	while (*ptr) {
		char ch = *ptr;
		if (ch != ' ')
			break;
		ptr++;
	}
	*inp = ptr;
}

bool Token::get(const char **inp, const char *sepStr, char *tok, int size, char *pSepBf) {
	return get(inp, sepStr, tok, size, pSepBf, NULL);
}

bool Token::get(const char **inp, const char *sepStr, char *tok, int size, char *pSepBf, char *pSepAf) {

	char mSepBf = 0;
	char mSepAf = 0;

	const char *ptr;

	ptr = *inp;

	while (*ptr) {
		char ch = *ptr;
		if (!isSep(sepStr, ch))
			break;
		if (mSepBf == 0) {
			mSepBf = ch;
		} else {
			if (mSepBf == ' ')
				mSepBf = ch;
		}
		ptr++;
	}

	bool inQuota = false;

	int k = 0;
	while (*ptr) {
		char ch = *ptr;
		if (!inQuota) {
			if (isSep(sepStr, ch)) {
				mSepAf = ch;
				break;
			}
		}

		if (ch == '\"') {
			inQuota = !inQuota;
		} else {
			if (k < size - 1) {
				tok[k++] = ch;
			}
		}
		ptr++;
	}
	tok[k] = 0;

	if (pSepBf != NULL)
		*pSepBf = mSepBf;
	if (pSepAf != NULL) {
		*pSepAf = mSepAf;
	}

	*inp = ptr;
	return (k != 0);
}

const char DEF_SEP[] = ",; =";
bool Token::get(const char **inp, char *tok, int size, char *pSepBf) {
	return get(inp, DEF_SEP, tok, size, pSepBf, NULL);
}

bool Token::get(const char **inp, char *tok, int size) {
	return get(inp, DEF_SEP, tok, size, NULL, NULL);
}

bool Token::get(const char **inp) {
	char buf[4];
	return get(inp, DEF_SEP, buf, sizeof(buf), NULL, NULL);
}

bool Token::getAsInt(const char **inp, int *val) {
	char tok[20];
	bool q = get(inp, DEF_SEP, tok, sizeof(tok), NULL, NULL);
	if (q) {
		*val = atoi(tok);
	}
	return q;
}

bool Token::getAsDate(const char **inp, TDATE *tm){
	char tok[20];
	bool q = get(inp, DEF_SEP, tok, sizeof(tok), NULL, NULL);
	if(q){
		const char *cmd = tok;
		q = TimeTools::parseDate(&cmd, tm);
	}
	return q;
}


bool Token::getAsBool(const char **inp, bool *val) {
	char tok[20];
	bool q = get(inp, DEF_SEP, tok, sizeof(tok), NULL, NULL);
	if (q) {
		int v = atoi(tok);
		*val = (v != 0);
	}
	return q;

}

//bool Token::get(const char **inp, const char *sepStr, char *tok, int size, char *pSep) {

bool Token::getAsInt(const char **inp, const char *sepStr, int *val) {
	char tok[20];
	bool q = get(inp, sepStr, tok, sizeof(tok), NULL, NULL);
	if (q) {
		*val = atoi(tok);
	}
	return q;

}

bool Token::getAsFloat(const char **inp, float *val) {
	char tok[20];
	bool q = get(inp, DEF_SEP, tok, sizeof(tok), NULL, NULL);
	if (q) {
		*val = atof(tok);
	}
	return q;
}

bool Token::chgCtrlChar(char *dst, const char *src, int max) {
	int k = 0;
	bool ret = false;

	while (1) {
		char ch = *src++;
		if (!ch) {
			ret = true;
			break;
		}
		switch (ch) {
		case '\n':
			if (k > max - 2)
				break;
			dst[k++] = '\\';
			dst[k++] = 'n';
			break;
		case '\r':
			if (k > max - 2)
				break;
			dst[k++] = '\\';
			dst[k++] = 'r';
			break;
		case '\\':
			if (k > max - 2)
				break;
			dst[k++] = '\\';
			dst[k++] = '\\';
			break;
		case '\e':
			if (k > max - 2)
				break;
			dst[k++] = '\\';
			dst[k++] = 'e';
			break;
		case '\t':
			if (k > max - 2)
				break;
			dst[k++] = '\\';
			dst[k++] = 't';
			break;
		default:
			if (ch < 0x20) {
				if (k > max - 5)
					break;
				dst[k++] = '\\';
				dst[k++] = '0';
				dst[k++] = 'x';
				char buf[4];
				snprintf(buf, sizeof(buf), "%02X", ch);
				dst[k++] = buf[0];
				dst[k++] = buf[1];

			} else {
				if (k > max - 1)
					break;
				dst[k++] = ch;
			}
			break;
		}
	}
	dst[k] = 0;
	return ret;
}

void Token::remooveEOL(char *line) {
	int n = strlen(line);
	while (n > 0) {
		char ch = line[n - 1];
		if (ch == '\n' || ch == '\r') {
			n--;
			line[n] = 0;
		} else
			break;
	}
}

bool Token::remooveQuota(char *line) {
	const char *ptr = line;

	trim(&ptr);
	int n = strlen(ptr);
	if (ptr[0] == '\"' && ptr[n - 1] == '\"') {
		n -= 2;
		memcpy(line, &ptr[1], n);
		line[n] = 0;
		return true;
	}
	return false;
}

void Token::copyNoQuota(char *dst, int max, const char *src) {

	int n = strlen(src);
	if (src[0] == '\"' && src[n - 1] == '\"') {
		n -= 2;
		if (n > max - 1)
			n = max - 1;
		memcpy(dst, &src[1], n);
		dst[n] = 0;
	}
}

void Token::setQuotaStr(char *dst, const char *txt, int max) {
	int n = strlen(txt);
	if (n > 0) {
		if (n > max - 2)
			n = max - 2;
		dst[0] = '\"';
		memcpy(&dst[1], txt, n);
		dst[1 + n] = '\"';
		dst[2 + n] = 0;
	} else {
		dst[0] = 0;
	}
}

void Token::shiftLeft(char *buf, int dist) {
	int n = strlen(buf);
	if (n > dist) {
		const char *src = &buf[dist];
		int i = 0;
		while (1) {
			buf[i] = src[i];
			if (!src[i])
				break;
			i++;
		}
	}
}

int Token::parseInt(const char *ptr, int len) {
	char buf[20];
	if (len < (int)sizeof(buf) - 1) {
		memcpy(buf, ptr, len);
		buf[len] = 0;
		return atoi(buf);
	}
	return -1;
}
