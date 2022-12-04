/**
 * This Library was originally written by Olivier Van den Eede (4ilo) in 2016.
 * Some refactoring was done and SPI support was added by Aleksander Alekseev (afiskon) in 2018.
 *
 * https://github.com/afiskon/stm32-ssd1306
 */

#ifndef __SSD1306_H__
#define __SSD1306_H__

#include <stddef.h>
#include <_ansi.h>
#include "I2cDev.h"

class SSD1306Dev: public I2c1Dev {
public:
	typedef enum {
		colBlack = 0x00, colWhite = 0x01
	} SSD1306_COLOR;

	typedef enum {
		fn6x8 = 0, fn7x10, fn11x18, fn16x26,
	} SSD1306_FONT;

private:
	enum {
		HEIGHT = 64, //
		WIDTH = 128, //
	};

	HAL_StatusTypeDef mError;
	uint8_t mBuffer[WIDTH * HEIGHT / 8]; // Screenbuffer
	char prnBuf[80];
	struct {
		uint16_t X;
		uint16_t Y;
		bool Inverted;
		bool Initialized;
		const void *pFont;
		SSD1306_COLOR color;

	} mState;
	void _WriteCmd(uint8_t cmd);
	void _WriteData(uint8_t *buffer, size_t buff_size);
	bool wrCharHd(char ch);
	const void* getFontDef(SSD1306_FONT font);
	void Fill(SSD1306_COLOR color);
protected:
	virtual void init();
	virtual void showState(MsgStream *strm);
	virtual bool isError(){
		return 0;
	}

public:
	SSD1306Dev(uint8_t adr);
	virtual void execFun(MsgStream *strm, int funNr);

	void initHd(void);
	void updateScr(void);
	void setFont(SSD1306_FONT font);
	void setColor(SSD1306_COLOR color);
	bool wrChar(char ch);
	bool wrStr(const char *str);
	void prn(const char *pFormat, ...);
	void drawPixel(uint8_t x, uint8_t y, SSD1306_COLOR color);
	void setCursor(uint8_t x, uint8_t y);
	void welcomeScr();
	void clear();
};

#endif // __SSD1306_H__
