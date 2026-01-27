
#include "font.h"
#include "main.h"
#include "typedef.h"
#include <stdint.h>

#define RST_PIN GPIOE, GPIO_PIN_5
#define DC_PIN GPIOE, GPIO_PIN_2
#define BL_PIN GPIOE, GPIO_PIN_9
#define CS_PIN GPIOE, GPIO_PIN_4

#define LCD_WIDTH 240  // LCD width
#define LCD_HEIGHT 280 // LCD height

/**
 * image color
 **/
#define WHITE 0xFFFF      // 0xFF 0xFF -> już OK
#define BLACK 0x0000      // 0x00 0x00
#define BLUE 0x1F00       // 0x00 0x1F
#define BRED 0x1FF8       // 0xF8 0x1F
#define GRED 0xE0FF       // 0xFF 0xE0
#define GBLUE 0xFF07      // 0x07 0xFF
#define RED 0x00F8        // 0xF8 0x00
#define MAGENTA 0x1FF8    // 0xF8 0x1F
#define GREEN 0xE007      // 0x07 0xE0
#define CYAN 0xFF7F       // 0x7F 0xFF
#define YELLOW 0xFFE0     // 0xE0 0xFF
#define BROWN 0x40BC      // 0xBC 0x40
#define BRRED 0x07FC      // 0xFC 0x07
#define GRAY 0x3084       // 0x84 0x30
#define DARKBLUE 0xCF01   // 0x01 0xCF
#define LIGHTBLUE 0x7C7D  // 0x7D 0x7C
#define GRAYBLUE 0x5854   // 0x54 0x58
#define LIGHTGREEN 0x1F84 // 0x84 0x1F
#define LGRAY 0x18C6      // 0xC6 0x18
#define LGRAYBLUE 0x51A6  // 0xA6 0x51
#define LBBLUE 0x122B     // 0x2B 0x12

extern SPI_HandleTypeDef hspi1;

void LCD_ScreenInit();
void LCD_ChangeBrightness(uint16_t value);
void LCD_FillScreen(uint16_t color);
void LCD_DrawImage(const uint16_t* pImage, int16_t xStart, int16_t yStart, int16_t W_Image, int16_t H_Image);
void LCD_FillRect(uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd, uint16_t Color);
void LCD_DrawString(int16_t xStart, int16_t yStart, const char* pString, Font_t* font, int16_t colorForeground, int16_t colorBackground);
void LCDScreenTask(void* argument);
