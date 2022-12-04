#include "string.h"
#include "stdarg.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "utils.h"

SSD1306Dev::SSD1306Dev(uint8_t adr) {
	mDevAdr = adr;
	init();
}

void SSD1306Dev::init() {
	memset(mBuffer, 0, sizeof(mBuffer));
	memset(&mState, 0, sizeof(mState));
	mDevExist = false;
	mError = HAL_OK;

	initHd();
	welcomeScr();
}

void SSD1306Dev::initHd(void) {
	mDevExist = (I2c1Bus::checkDevMtx(mDevAdr) == HAL_OK);
	if (!mDevExist)
		return;

	if (I2c1Bus::openMutex(50, 100)) {

		// initHd OLED

		_WriteCmd(0xAE); //display off

		_WriteCmd(0x20); //Set Memory Addressing Mode
		_WriteCmd(0x00); // 00b,Horizontal Addressing Mode; 01b,Vertical Addressing Mode;
						 // 10b,Page Addressing Mode (RESET); 11b,Invalid

		_WriteCmd(0xB0); //Set Page Start Address for Page Addressing Mode,0-7

#ifdef SSD1306_MIRROR_VERT
    _WriteCmd(0xC0); // Mirror vertically
#else
		_WriteCmd(0xC8); //Set COM Output Scan Direction
#endif

		_WriteCmd(0x00); //---set low column address
		_WriteCmd(0x10); //---set high column address

		_WriteCmd(0x40); //--set start line address - CHECK

		_WriteCmd(0x81); //--set contrast control register - CHECK
		_WriteCmd(0xFF);

#ifdef SSD1306_MIRROR_HORIZ
    _WriteCmd(0xA0); // Mirror horizontally
#else
		_WriteCmd(0xA1); //--set segment re-map 0 to 127 - CHECK
#endif

#ifdef SSD1306_INVERSE_COLOR
    _WriteCmd(0xA7); //--set inverse color
#else
		_WriteCmd(0xA6); //--set normal color
#endif

		_WriteCmd(0xA8); //--set multiplex ratio(1 to 64) - CHECK
		_WriteCmd(0x3F); //

		_WriteCmd(0xA4); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content

		_WriteCmd(0xD3); //-set display offset - CHECK
		_WriteCmd(0x00); //-not offset

		_WriteCmd(0xD5); //--set display clock divide ratio/oscillator frequency
		_WriteCmd(0xF0); //--set divide ratio

		_WriteCmd(0xD9); //--set pre-charge period
		_WriteCmd(0x22); //

		_WriteCmd(0xDA); //--set com pins hardware configuration - CHECK
		_WriteCmd(0x12);

		_WriteCmd(0xDB); //--set vcomh
		_WriteCmd(0x20); //0x20,0.77xVcc

		_WriteCmd(0x8D); //--set DC-DC enable
		_WriteCmd(0x14); //
		_WriteCmd(0xAF); //--turn on SSD1306 panel

		// Clear screen
		Fill(colWhite);

		// Flush buffer to screen
		updateScr();

		// Set default values for screen object
		mState.X = 0;
		mState.Y = 0;

		mState.Initialized = 1;
		setFont(fn7x10);
		setColor(colWhite);

		I2c1Bus::closeMutex();
	}
}


void SSD1306Dev::showState(MsgStream *strm) {
	strm->msgItem("__SSD1306__");
	strm->msgItem("chipExist: %s", YN(mDevExist));
	if (mDevExist) {
		strm->msgItem("mError: %s", HAL_getErrStr(mError));
	}
}

void SSD1306Dev::execFun(MsgStream *strm, int funNr) {
	switch (funNr) {
	case 30:
		mError = HAL_OK;
		mState.Initialized = 0;
		initHd();
		strm->msgItem("SSD1306 Init st=%s", HAL_getErrStr(mError));
		strm->msgItem("Initialized=%s", YN(mState.Initialized));
		break;
	case 31:
		clear();
		prn("Temp:%.2f[%%]\nPres:%.1f[kPa]", 23.45, 987.23);
		updateScr();
		break;
	}
}

void SSD1306Dev::_WriteCmd(uint8_t cmd) {
	HAL_StatusTypeDef st = I2c1Bus::writeByte(mDevAdr, 0x00, cmd);
	if (st != HAL_OK)
		mError = st;
}

void SSD1306Dev::_WriteData(uint8_t *buffer, size_t buff_size) {
	HAL_StatusTypeDef st = I2c1Bus::writeBytes(mDevAdr, 0x40, buff_size, buffer);
	if (st != HAL_OK)
		mError = st;
}


void SSD1306Dev::Fill(SSD1306_COLOR color) {
	int v = (color == colBlack) ? 0x00 : 0xFF;
	memset(mBuffer, v, sizeof(mBuffer));
}

void SSD1306Dev::setCursor(uint8_t x, uint8_t y) {
	mState.X = x;
	mState.Y = y;
}

void SSD1306Dev::updateScr(void) {
	if (mDevExist) {
		if (I2c1Bus::openMutex(51, 100)) {

			for (int i = 0; i < 8; i++) {
				_WriteCmd(0xB0 + i);
				_WriteCmd(0x00);
				_WriteCmd(0x10);
				_WriteData(&mBuffer[WIDTH * i], WIDTH);
			}
			I2c1Bus::closeMutex();
		}
	}
}

void SSD1306Dev::drawPixel(uint8_t x, uint8_t y, SSD1306_COLOR color) {
	if (x >= WIDTH || y >= HEIGHT) {
		return;
	}

	if (mState.Inverted) {
		color = (SSD1306_COLOR) !color;
	}

	if (color == colWhite) {
		mBuffer[x + (y / 8) * WIDTH] |= 1 << (y % 8);
	} else {
		mBuffer[x + (y / 8) * WIDTH] &= ~(1 << (y % 8));
	}
}

const void* SSD1306Dev::getFontDef(SSD1306_FONT font) {
	switch (font) {
	default:
	case fn6x8:
		return &Font_6x8;
	case fn7x10:
		return &Font_7x10;
	case fn11x18:
		return &Font_11x18;
	case fn16x26:
		return &Font_16x26;
	}
}

void SSD1306Dev::setFont(SSD1306_FONT font) {
	mState.pFont = getFontDef(font);
}
void SSD1306Dev::setColor(SSD1306_COLOR color) {
	mState.color = color;
}

bool SSD1306Dev::wrCharHd(char ch) {
	FontDef *pFont = (FontDef*) mState.pFont;

	if (ch != '\n') {
		if (ch < 32 || ch > 126)
			return false;

		if ((mState.X >= WIDTH) || (mState.Y >= HEIGHT)) {
			return false;
		}

		for (int i = 0; i < pFont->FontHeight; i++) {
			uint32_t b = pFont->data[(ch - 32) * pFont->FontHeight + i];
			for (int j = 0; j < pFont->FontWidth; j++) {
				if ((b << j) & 0x8000) {
					drawPixel(mState.X + j, (mState.Y + i), (SSD1306_COLOR) mState.color);
				} else {
					drawPixel(mState.X + j, (mState.Y + i), (SSD1306_COLOR) !mState.color);
				}
			}
		}
		mState.X += pFont->FontWidth;
	} else {
		mState.X = 0;
		mState.Y += pFont->FontHeight;
	}

	return true;
}

bool SSD1306Dev::wrChar(char ch) {
	return wrCharHd(ch);
}

bool SSD1306Dev::wrStr(const char *str) {
	while (*str) {
		wrCharHd(*str);
		str++;
	}
	return true;
}

void SSD1306Dev::prn(const char *pFormat, ...) {
	va_list ap;
	va_start(ap, pFormat);
	vsnprintf(prnBuf, sizeof(prnBuf), pFormat, ap);
	//strncpy(prnBuf,pFormat,sizeof(prnBuf));
	va_end(ap);
	wrStr(prnBuf);
}

void SSD1306Dev::clear() {
	mState.X = 0;
	mState.Y = 0;
	Fill(colBlack);
}

void SSD1306Dev::welcomeScr() {
	clear();
	setFont(fn16x26);
	wrStr("AIR-PRO\n");
	setFont(fn7x10);
	wrStr("\nMQTT-BG96");
	updateScr();
}
