/*
 * EscTerminal.cpp
 *
 *  Created on: Dec 5, 2020
 *      Author: Grzegorz
 */
/*
 * st.UP     27 91 65 ESC[A
 * st.DN     27 91 66 ESC[B
 * st.RIGHT  27 91 67 ESC[C
 * st.LEFT   27 91 68 ESC[D
 *
 *Home		 27 91 49 126 ESC[1 x
 *Insert         27 91 50 126 ESC[2 x
 *Delete	 27 91 51 126 ESC[3 x
 *End		 27 91 52 126 ESC[4 x
 *PgUp		 27 91 53 126 ESC[5 x
 *PgDn		 27 91 54 126 ESC[6 x
 *
 * F1   27 91 49 49 126 ESC[11 x
 * F2   27 91 49 50 126
 * F3	27 91 49 51 126
 * F4	27 91 49 52 126
 * F5	27 91 49 53 126 ESC[15 x
 * F6	27 91 49 55 126 ESC[17 x
 * F7	27 91 49 56 126
 * F8	27 91 49 57 126 ESC[19 x
 * F9	27 91 50 48 126 ESC[20 x
 * F10	27 91 50 49 126 ESC[21 x
 * F11	27 91 50 51 126 ESC[23 x
 * F12	27 91 50 52 126 ESC[24 x
 *
 *
 *
 *
 */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include <EscTerminal.h>

OutHdStream::OutHdStream() {
	mNoTermSmfCnt = 0;
}
OutHdStream::~OutHdStream() {

}


void OutHdStream::escTermShowLineNoMx() {
	escTerminal->showLineNoMx();
}

void OutHdStream::putChar(char ch) {
	putOut(&ch, 1);
}

void OutHdStream::putStr(const char *txt) {
	putOut(txt, strlen(txt));
}

void OutHdStream::putCharMx(char ch) {
	if (openOutMutex(STD_TIME)) {
		putChar(ch);
		closeOutMutex();
	}
}

void OutHdStream::putStrMx(const char *txt) {
	if (openOutMutex(STD_TIME)) {
		putStr(txt);
		closeOutMutex();
	}
}


//----------------------------------------------------------------------------------------
HistCmd::HistCmd() {
	memset(mem, 0, sizeof(mem));
	viewPtr = 0;
}

void HistCmd::push(const char *line) {
	if (line[0] == 0)
		return;
	// sprawdzenie czy jest juz taka linia
	int fndWsk = -1;
	int wsk = 0;
	while (1) {
		int n = mem[wsk];
		if (n == 0 || wsk + n > HIST_SIZE) {
			break;
		}
		const char *s1 = &mem[wsk + 1];
		if (strcmp(line, s1) == 0) {
			fndWsk = wsk;
			break;
		}
		wsk += (n + 1);
	}

	if (fndWsk >= 0) {
		//usunięcie ze śroka linii
		int n = mem[fndWsk] + 1;
		int k = fndWsk;
		while (k + n < HIST_SIZE) {
			mem[k] = mem[k + n];
			k++;
		}
		while (k < HIST_SIZE) {
			mem[k] = 0;
			k++;
		}
	}

	//zrobienie miejsca na początku
	int n = strlen(line) + 1; //razem z zero
	int i = HIST_SIZE - (n + 1);
	while (i > 0) {
		i--;
		mem[n + i + 1] = mem[i];
	}
	//skopiowanie na początek
	memcpy(&mem[1], line, n);
	mem[0] = n;
	viewPtr = 0;
}

const char* HistCmd::popPrev() {
	const char *ptr = NULL;
	int n = mem[viewPtr];
	if (n != 0 && viewPtr + n < HIST_SIZE) {
		ptr = &mem[viewPtr + 1];
		viewPtr += (n + 1);
	}
	return ptr;
}

const char* HistCmd::popNext() {
	const char *ptr = NULL;
	if (viewPtr > 0) {
		int wsk = 0;
		while (1) {
			int n = mem[wsk];
			if (n == 0 || wsk + n > HIST_SIZE) {
				break;
			}
			int oWsk = wsk;
			wsk += (n + 1);
			if (wsk == viewPtr) {
				viewPtr = oWsk;
				ptr = &mem[viewPtr + 1];
				break;
			}
		}
	}
	return ptr;
}

//----------------------------------------------------------------------------------------

EditLine::EditLine(OutHdStream *outStream) {
	mOut = outStream;
	mPtr = 0;
	mCurs = 0;
	insMode = true;
	mPrompt[0] = 0;
	mPromptLen = 0;
}
void EditLine::setPrompt(const char *pr) {
	strncpy(mPrompt, pr, sizeof(mPrompt));
	mPrompt[sizeof(mPrompt) - 1] = 0;
	mPromptLen = strlen(mPrompt);
}

void EditLine::sendCursPos() {
	char buf[10];
	int n = snprintf(buf, sizeof(buf), "\e[%uG", mCurs + 1 + mPromptLen);
	mOut->putOut(buf, n);
}
void EditLine::sendCursPosMx() {
	if (mOut->openOutMutex(OutHdStream::STD_TIME)) {
		sendCursPos();
		mOut->closeOutMutex();
	}
}

void EditLine::sendCLine() {
	mOut->putStr(TERM_CLEAR_LINE);
	if (!insMode)
		mOut->putStr(TERM_COLOR_MAGENTA);
	else
		mOut->putStr(TERM_COLOR_CYAN);

	mOut->putStr(mPrompt);
	mOut->putOut(mBuf, mPtr);
	if (mCurs != mPtr) {
		sendCursPos();
	}
}

void EditLine::sendCLineMx() {
	if (mOut->openOutMutex(OutHdStream::STD_TIME)) {
		sendCLine();
		mOut->closeOutMutex();
	}
}

void EditLine::addChar(char ch) {

	if (mCurs == mPtr) {
		//pisanie na końcu linii
		if (mPtr < MAX_LINE_SIZE) {
			mBuf[mPtr] = ch;
			mPtr++;
			mCurs++;
			mOut->putCharMx(ch);
		}
	} else {
		//pisanie w środku linii
		if (!insMode) {
			mBuf[mCurs] = ch;
			mCurs++;
			mOut->putCharMx(ch);
		} else {
			if (mCurs < MAX_LINE_SIZE) {
				int k = mPtr;
				if (k == MAX_LINE_SIZE)
					k--;
				k--;

				for (int i = k; i >= mCurs; i--) {
					mBuf[i + 1] = mBuf[i];
				}
				mBuf[mCurs] = ch;
				mCurs++;
				if (mPtr < MAX_LINE_SIZE)
					mPtr++;
			}
			sendCLineMx();
		}

	}
}

void EditLine::getCmd(char *cmd) {
	memcpy(cmd, mBuf, mPtr);
	cmd[mPtr] = 0;
}

void EditLine::bkSpace() {
	if (mCurs > 0) {
		mPtr--;
		mCurs--;
		if (mPtr != mCurs) {
			for (int i = mCurs; i < mPtr; i++) {
				mBuf[i] = mBuf[i + 1];
			}
			sendCLineMx();
		} else {
			char buf[40];
			int k = mCurs + 1 + mPromptLen;
			snprintf(buf, sizeof(buf), "\e[%uG \e[%uG", k, k);
			mOut->putStrMx(buf);
		}

	}
}
void EditLine::altBkSpace() {
	if (mCurs > 0) {
		if (mPtr != mCurs) {
			for (int i = 0; i < mPtr - mCurs; i++) {
				mBuf[i] = mBuf[mCurs + i];
			}
		}
		mPtr -= mCurs;
		mCurs = 0;
		sendCLineMx();
	}
}

void EditLine::moveLeft() {
	if (mCurs > 0) {
		mCurs--;
		sendCursPosMx();
	}
}
void EditLine::moveRight() {
	if (mCurs < mPtr) {
		mCurs++;
		sendCursPosMx();
	}
}

void EditLine::chMode() {
	insMode = !insMode;
	sendCLineMx();
}

void EditLine::deleteChar() {
	if (mCurs < mPtr) {
		for (int i = mCurs; i < mPtr; i++) {
			mBuf[i] = mBuf[i + 1];
		}
		mPtr--;
		sendCLineMx();
	}
}

void EditLine::altDeleteChar() {
	mPtr = mCurs;
	sendCLineMx();
}

void EditLine::home() {
	mCurs = 0;
	sendCursPosMx();
}

void EditLine::end() {
	mCurs = mPtr;
	sendCursPosMx();

}

void EditLine::setBuf(const char *src) {
	if (src != NULL) {
		int n = strlen(src);
		strncpy(mBuf, src, sizeof(mBuf));
		mPtr = n;
		mCurs = n;
	} else {
		mPtr = 0;
		mCurs = 0;
	}
	sendCLineMx();
}

void EditLine::newLine() {
	mPtr = 0;
	mCurs = 0;
	mOut->putStrMx("\r\n");
}

//----------------------------------------------------------------------------------------

CsiEng::CsiEng() {
	clear();
}

bool CsiEng::isIdle() {
	return seq == csiIDLE;
}

void CsiEng::clear() {
	seq = csiIDLE;
	params[0] = 0;
	inter[0] = 0;
	mAltFun = false;
}

void CsiEng::start() {
	seq = csiPAR;
}

void CsiEng::strAdd(char *buf, int size, char ch) {
	int n = strlen(buf);
	if (n < size - 1) {
		buf[n] = ch;
		buf[n + 1] = 0;
	}
}

bool CsiEng::putChar(char ch) {
	if (ch >= 0x30 && ch < 0x40) {
		if (seq == csiPAR) {
			strAdd(params, sizeof(params), ch);
		}
	} else if (ch >= 0x20 && ch < 0x30) {
		if (seq == csiPAR || seq == csiINTER) {
			strAdd(inter, sizeof(inter), ch);
			seq = csiINTER;
		}
	} else if (ch >= 0x40 && ch < 0x7F) {
		finalChar = ch;
		seq = csiFINAL;
		return true;
	}
	return false;
}

const int fnCodeTab[12] = { 11, 12, 13, 14, 15, 17, 18, 19, 20, 21, 23, 24 };

FunKey CsiEng::getFunKey() {

	switch (finalChar) {
	case 'A':
		return fnUP;
	case 'B':
		return fnDN;
	case 'C':
		return fnRIGHT;
	case 'D':
		return fnLEFT;
	case 0x7E: {
		int par = atoi(params);
		FunKey fun = fnNULL;
		if (par >= 1 && par <= 6) {
			fun = (FunKey) (fnHOME + (par - 1));
		} else if (par >= 11 && par <= 24) {  //F1..F12
			for (int i = 0; i < 12; i++) {
				if (fnCodeTab[i] == par) {
					fun = (FunKey) (fnF1 + i);
					break;
				}
			}
		}
		if (mAltFun && fun != fnNULL) {
			if (fun >= fnF1 && fun <= fnF12) {
				fun = (FunKey) (fun - fnF1 + fnAltF1);
			} else if (fun == fnDELETE) {
				fun = fnAltDELETE;
			}
		}
		return fun;

	}
		break;
	}
	return fnNULL;
}

//----------------------------------------------------------------------------------------

EscTerminal::EscTerminal(OutHdStream *outStream) {
	mOut = outStream;
	outStream->escTerminal = this;

	cLine = new EditLine(mOut);
	mHistCmd = new HistCmd();
	csi = new CsiEng();
	mWasESC = false;
	setStdPrompt();
}
void EscTerminal::showLineNoMx() {
	cLine->sendCLine();
}
void EscTerminal::showLineMx(){
	cLine->sendCLineMx();
}

void EscTerminal::setStdPrompt() {
	setPrompt("$>");
}

void EscTerminal::setPrompt(const char *txt) {
	cLine->setPrompt(txt);
	cLine->sendCLineMx();
}

TermAct EscTerminal::inpChar(char ch) {
	if (!mWasESC) {
		if (ch == 27) {
			mWasESC = true;
			csi->clear();
		} else {
			if (ch == 8 || ch == 127) {
				cLine->bkSpace();
			} else if (ch == 10) {

			} else if (ch == 13) {
				cLine->getCmd(mCmd);
				cLine->newLine();

				mHistCmd->push(mCmd);

				return actLINE;
			} else {
				//zwykły znak
				cLine->addChar(ch);
			}
		}
	} else {
		if (csi->isIdle()) {
			if (ch == '[') {
				csi->start();
			} else if (ch == 27) {
				csi->setAltFun();
			} else if (ch == 8 || ch == 127) {
				cLine->altBkSpace();
				mWasESC = false;
			} else {
				mFunKey = fnNULL;
				mAltChar = ch;
				mWasESC = false;
				return actALTCHAR;
			}
		} else {
			if (csi->putChar(ch)) {
				FunKey fun = csi->getFunKey();
				csi->clear();
				mWasESC = false;
				if (fun < fnF1) {
					switch (fun) {
					case fnUP: {
						const char *ln = mHistCmd->popPrev();
						cLine->setBuf(ln);
					}
						break;
					case fnDN: {
						const char *ln = mHistCmd->popNext();
						cLine->setBuf(ln);
					}
						break;
					case fnLEFT:
						cLine->moveLeft();
						break;
					case fnRIGHT:
						cLine->moveRight();
						break;
					case fnINSERT:
						cLine->chMode();
						break;
					case fnDELETE:
						cLine->deleteChar();
						break;
					case fnAltDELETE:
						cLine->altDeleteChar();
						break;
					case fnHOME:
						cLine->home();
						break;
					case fnEND:
						cLine->end();
						break;
					case fnPGUP:
						break;
					case fnPGDN:
						break;
					default:
						break;
					}
				} else {
					mFunKey = fun;
					mAltChar = 0;
					return actFUNKEY;
				}

			}
		}

	}
	return actNOTHING;

}

const char* EscTerminal::getColorStr(TermColor color) {
	switch (color) {
	default:
	case colWHITE:
		return TERM_COLOR_WHITE;
	case colRED:
		return TERM_COLOR_RED;
	case colGREEN:
		return TERM_COLOR_GREEN;
	case colBLUE:
		return TERM_COLOR_BLUE;
	case colMAGENTA:
		return TERM_COLOR_MAGENTA;
	case colYELLOW:
		return TERM_COLOR_YELLOW;
	case colCYAN:
		return TERM_COLOR_CYAN;
	}
}
