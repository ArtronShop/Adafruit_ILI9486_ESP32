/*
See rights and use declaration in License.h
This library has been modified for the Maple Mini

-----------
modified by IOXhop (www.ioxhop.com)
 - Add micro MISO MOSI and SCK pin
-----------
*/

#ifndef _ADAFRUIT_ILI9486H_
#define _ADAFRUIT_ILI9486H_

//#include "Arduino.h"
//#include "Print.h"
#include <Adafruit_GFX.h>
#include <SPI.h> // Using library SPI in folder: D:\Documents\Arduino\hardware\Arduino_STM32\STM32F1\libraries\SPI

//#define USE_DMA 1
extern uint8_t useDMA;

#define TFTWIDTH	320
#define TFTHEIGHT	480

#define ILI9486_INVOFF	0x20
#define ILI9486_INVON	0x21
#define ILI9486_CASET	0x2A
#define ILI9486_PASET	0x2B
#define ILI9486_RAMWR	0x2C
#define ILI9486_MADCTL	0x36
#define MADCTL_MY  0x80
#define MADCTL_MX  0x40
#define MADCTL_MV  0x20
#define MADCTL_ML  0x10
#define MADCTL_BGR 0x08
#define MADCTL_MH  0x04

// Color definitions
#define BLACK       0x0000      /*   0,   0,   0 */
#define NAVY        0x000F      /*   0,   0, 128 */
#define DARKGREEN   0x03E0      /*   0, 128,   0 */
#define DARKCYAN    0x03EF      /*   0, 128, 128 */
#define MAROON      0x7800      /* 128,   0,   0 */
#define PURPLE      0x780F      /* 128,   0, 128 */
#define OLIVE       0x7BE0      /* 128, 128,   0 */
#define LIGHTGREY   0xC618      /* 192, 192, 192 */
#define DARKGREY    0x7BEF      /* 128, 128, 128 */
#define BLUE        0x001F      /*   0,   0, 255 */
#define GREEN       0x07E0      /*   0, 255,   0 */
#define CYAN        0x07FF      /*   0, 255, 255 */
#define RED         0xF800      /* 255,   0,   0 */
#define MAGENTA     0xF81F      /* 255,   0, 255 */
#define YELLOW      0xFFE0      /* 255, 255,   0 */
#define WHITE       0xFFFF      /* 255, 255, 255 */
#define ORANGE      0xFD20      /* 255, 165,   0 */
#define GREENYELLOW 0xAFE5      /* 173, 255,  47 */
#define PINK        0xF81F

/*
Define pins and Output Data Registers
*/

//Control pins |RS |CS |RST|
#define TFT_RST        26
#define TFT_RS         27
#define TFT_CS         25

// ESP32
#define TFT_SCK 14
#define TFT_MISO 12
#define TFT_MOSI 13

// extern gpio_reg_map *ctrlRegs;
// #define CS_ACTIVE    ctrlRegs->BRR  = TFT_CS_MASK //digitalWrite(TFT_CS, LOW); //
// #define CS_IDLE      ctrlRegs->BSRR = TFT_CS_MASK //digitalWrite(TFT_CS, HIGH); //
// #define CD_COMMAND   ctrlRegs->BRR  = TFT_RS_MASK //digitalWrite(TFT_RS, LOW); //
// #define CD_DATA      ctrlRegs->BSRR = TFT_RS_MASK //digitalWrite(TFT_RS, HIGH); //
/*#define CS_ACTIVE    digitalWrite(TFT_CS, LOW);
#define CS_IDLE      digitalWrite(TFT_CS, HIGH);
#define CD_COMMAND   digitalWrite(TFT_RS, LOW);
#define CD_DATA      digitalWrite(TFT_RS, HIGH);*/
#define CS_ACTIVE    gpio_set_level((gpio_num_t)TFT_CS, 0);
#define CS_IDLE      gpio_set_level((gpio_num_t)TFT_CS, 1);
#define CD_COMMAND   gpio_set_level((gpio_num_t)TFT_RS, 0);
#define CD_DATA      gpio_set_level((gpio_num_t)TFT_RS, 1);

#define swap(a, b) { int16_t t = a; a = b; b = t; }

/*****************************************************************************/
class Adafruit_ILI9486_ESP32 : public Adafruit_GFX
{
public:

	Adafruit_ILI9486_ESP32(void);
  
	void	begin(void),
			setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1),
			pushColor(uint16_t color),
			fillScreen(uint16_t color),
			drawLine(int16_t x0, int16_t y0,int16_t x1, int16_t y1, uint16_t color),
			drawPixel(int16_t x, int16_t y, uint16_t color),
			drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color),
			drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color),
			fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color),
			setRotation(uint8_t r),
			invertDisplay(boolean i);
	uint16_t color565(uint8_t r, uint8_t g, uint8_t b);

	void reset(void);

 private:
	uint8_t	tabcolor;
};
	void	writecommand(uint8_t c),
			writedata(uint8_t d),
			writedata16(uint16_t d),
			writedata16(uint16_t d, uint32_t num),
			commandList(uint8_t *addr);

#endif //endif of the header file
