/*
 * EscTerminal.h
 *
 *  Created on: Dec 5, 2020
 *      Author: Grzegorz
 */

#ifndef ESCTERMINAL_H_
#define ESCTERMINAL_H_

class OutHdStream {
	friend class EscTerminal;
	friend class EditLine;
private:
	int mNoTermSmfCnt;

public:
	OutHdStream();
	virtual ~OutHdStream();
	enum {
		STD_TIME = 1000,
	};
	virtual bool openOutMutex(int tm){
		return true;
	}
	virtual void closeOutMutex(){

	}
	virtual void putOut(const void *mem, int len) = 0;
	void putChar(char ch);
	void putStr(const char *txt);
	void putStrMx(const char *txt);
	void putCharMx(char ch);
};

//https://en.wikipedia.org/wiki/ANSI_escape_code

#define TERM_CLEAR_SCR	    "\e[2J\e[1;1H\e[0m"       //wyczyszczenie ekranu
#define TERM_CLEAR_LINE     "\e[2K\e[1G"  //wyczyszczenie linii i ustawienie kursora na początku
#define TERM_COLOR_GREEN    "\e[32m"
#define TERM_COLOR_BLUE     "\e[34m"
#define TERM_COLOR_MAGENTA  "\e[35m"
#define TERM_COLOR_CYAN     "\e[36m"
#define TERM_COLOR_WHITE    "\e[97m"
#define TERM_COLOR_YELLOW   "\e[93m"
#define TERM_COLOR_RED	    "\e[91m"

typedef enum {
	colWHITE = 0, colRED, colGREEN, colBLUE, colMAGENTA, colYELLOW, colCYAN,
} TermColor;

enum {
	MAX_LINE_SIZE = 120,
};

class HistCmd {
private:
	enum {
		HIST_SIZE= 400,
	};
	int viewPtr;
	char mem[HIST_SIZE];
public:
	HistCmd();
	void push(const char *line);
	const char* popPrev();
	const char* popNext();
};

class EditLine {
private:
	OutHdStream *mOut;
	bool insMode;
	int mPtr;
	int mCurs; //pozycja kursora
	char mBuf[MAX_LINE_SIZE];
	char mPrompt[4];
	int mPromptLen;
	void sendCursPos();
	void sendCursPosMx();
public:
	EditLine(OutHdStream *outStream);
	void sendCLine();
	void sendCLineMx();


	void setPrompt(const char *pr);
	void bkSpace();
	void altBkSpace();
	void getCmd(char *cmd);
	void newLine();
	void addChar(char ch);
	void setBuf(const char *src);

	void moveLeft();
	void moveRight();
	void chMode();
	void deleteChar();
	void altDeleteChar();

	void home();
	void end();
};

typedef enum {
	fnNULL = 0, //
	fnUP, //
	fnDN, //
	fnLEFT, //
	fnRIGHT, //

	fnHOME, //
	fnINSERT, //
	fnDELETE, //
	fnEND, //
	fnPGUP, //
	fnPGDN, //
	fnAltDELETE,

	fnF1 = 21, //
	fnF2, //
	fnF3, //
	fnF4, //
	fnF5, //
	fnF6, //
	fnF7, //
	fnF8, //
	fnF9, //
	fnF10, //
	fnF11, //
	fnF12, //
	fnAltF1 = 41, //
	fnAltF2, //
	fnAltF3, //
	fnAltF4, //
	fnAltF5, //
	fnAltF6, //
	fnAltF7, //
	fnAltF8, //
	fnAltF9, //
	fnAltF10, //
	fnAltF11, //
	fnAltF12, //
} FunKey;

typedef enum {
	csiIDLE = 0, //
	csiPAR, //
	csiINTER, //
	csiFINAL, //
} CsiSeq;

class CsiEng {
private:
	CsiSeq seq;
	char params[6];
	char inter[6];
	char finalChar;
	bool mAltFun;
	void strAdd(char *buf, int size, char ch);
public:

	CsiEng();
	void clear();
	void start();
	bool putChar(char ch);
	bool isIdle();
	void setAltFun() {
		mAltFun = true;
	}

	FunKey getFunKey();
};

typedef enum {
	actNOTHING = 0, actLINE, actALTCHAR, actFUNKEY
} TermAct;

class EscTerminal {
private:
	OutHdStream *mOut;
	HistCmd *mHistCmd;
	EditLine *cLine;
	CsiEng *csi;
	bool mWasESC;
public:
	char mCmd[MAX_LINE_SIZE + 1];
	char mAltChar;
	FunKey mFunKey;
	EscTerminal(OutHdStream *outStream);
	TermAct inpChar(char ch);
	void showLineNoMx();
	void showLineMx();
	static const char* getColorStr(TermColor color);

};

#endif /* ESCTERMINAL_H_ */
