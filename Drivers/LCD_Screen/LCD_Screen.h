
#include <stdint.h>
#include "main.h"
#include "font.h"
#include "typedef.h"

#define RST_PIN GPIOE, GPIO_PIN_5
#define DC_PIN GPIOE, GPIO_PIN_2
#define BL_PIN GPIOE, GPIO_PIN_9
#define CS_PIN GPIOE, GPIO_PIN_4

#define LCD_WIDTH 240  // LCD width
#define LCD_HEIGHT 280 // LCD height


/**
 * image color
**/
#define WHITE         0xFFFF
#define BLACK         0x0000
#define BLUE          0x001F
#define BRED          0XF81F
#define GRED          0XFFE0
#define GBLUE         0X07FF
#define RED           0xF800
#define MAGENTA       0xF81F
#define GREEN         0x07E0
#define CYAN          0x7FFF
#define YELLOW        0xFFE0
#define BROWN         0XBC40
#define BRRED         0XFC07
#define GRAY          0X8430
#define DARKBLUE      0X01CF
#define LIGHTBLUE     0X7D7C
#define GRAYBLUE      0X5458
#define LIGHTGREEN    0X841F
#define LGRAY         0XC618
#define LGRAYBLUE     0XA651
#define LBBLUE        0X2B12

extern SPI_HandleTypeDef hspi1;

void setPWM_PE9(uint16_t value);
void Clear(uint16_t color);
void SetPixel(uint16_t x, uint16_t y, uint16_t color);
void DrawImage(const uint8_t* pImage, int16_t xStart, int16_t yStart, int16_t W_Image, int16_t H_Image);
void ScreenInit();
void ClearWindow(uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd, uint16_t color);
void SetWindowColor(uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd, uint16_t Color);
void SetCursor(uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd);
void DrawString(int16_t xStart, int16_t yStart, const char * pString, Font_t* font, int16_t colorBackground, int16_t colorForeround, uint8_t useBackgroundColor);
void DrawChar(int16_t xPoint, int16_t yPoint, const char asciiChar, Font_t* font, int16_t colorBackground, int16_t colorForeround, uint8_t useBackgroundColor);
