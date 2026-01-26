/*
 * LCD_Screen.c
 *
 * Driver for ST7789V2 LCD Screen
 * This code is based on
 * https://github.com/limengdu/XIAO_ST7789V2_LCD_Display
 *
 * Ported to STM32 Cube HAL by Wojciech Niewiadomski
 *
 *  Created on: Jan 21, 2026
 *
 */

#include "LCD_Screen.h"

#define LCD_CS_LOW() HAL_GPIO_WritePin(CS_PIN, GPIO_PIN_RESET)
#define LCD_CS_HIGH() HAL_GPIO_WritePin(CS_PIN, GPIO_PIN_SET)

#define LCD_DC_LOW() HAL_GPIO_WritePin(DC_PIN, GPIO_PIN_RESET)
#define LCD_DC_HIGH() HAL_GPIO_WritePin(DC_PIN, GPIO_PIN_SET)

#define LCD_RST_LOW() HAL_GPIO_WritePin(RST_PIN, GPIO_PIN_RESET)
#define LCD_RST_HIGH() HAL_GPIO_WritePin(RST_PIN, GPIO_PIN_SET)

static void WriteData(uint8_t* data, uint32_t size);
static void WriteData_Byte(uint8_t data);
static void WriteReg(uint8_t data);

static void WriteData(uint8_t* data, uint32_t size)
{
    LCD_DC_HIGH();
    LCD_CS_LOW();
    HAL_SPI_Transmit(&hspi1, data, size, HAL_MAX_DELAY);
    LCD_CS_HIGH();
}

static void WriteData_Byte(uint8_t data)
{
    LCD_DC_HIGH();
    LCD_CS_LOW();
    HAL_SPI_Transmit(&hspi1, (uint8_t*)&data, 1, HAL_MAX_DELAY);
    LCD_CS_HIGH();
}

static void WriteReg(uint8_t data)
{
    LCD_DC_LOW();
    LCD_CS_LOW();
    HAL_SPI_Transmit(&hspi1, (uint8_t*)&data, 1, HAL_MAX_DELAY);
    LCD_CS_HIGH();
}

void SetCursor(uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd)
{
    WriteReg(0x2A);
    WriteData_Byte(xStart >> 8);
    WriteData_Byte(xStart);
    WriteData_Byte((xEnd) >> 8);
    WriteData_Byte(xEnd);
    // set the Y coordinates
    WriteReg(0x2B);
    WriteData_Byte((yStart + 20) >> 8);
    WriteData_Byte(yStart + 20);
    WriteData_Byte((yEnd + 20) >> 8);
    WriteData_Byte(yEnd + 20);
    WriteReg(0X2C);
}

void SetWindowColor(uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd, uint16_t color)
{

    static uint16_t lineBuf[LCD_WIDTH];

    uint16_t w = xEnd - xStart;
    uint16_t h = yEnd - yStart;

    for (uint32_t idx = 0u; idx < w; idx++)
    {
        lineBuf[idx] = color;
    }

    SetCursor(xStart, yStart, xEnd, yEnd);

    LCD_DC_HIGH();
    LCD_CS_LOW();

    for (uint32_t row = 0u; row < h; row++)
    {
        HAL_SPI_Transmit(&hspi1, (uint8_t*)lineBuf, w * 2, HAL_MAX_DELAY);
    }

    LCD_CS_HIGH();
}

void ScreenReset()
{
    HAL_GPIO_WritePin(CS_PIN, GPIO_PIN_RESET);
    HAL_Delay(20);
    HAL_GPIO_WritePin(RST_PIN, GPIO_PIN_RESET);
    HAL_Delay(20);
    HAL_GPIO_WritePin(RST_PIN, GPIO_PIN_SET);
    HAL_Delay(20);
}

void ScreenInit()
{
    LCD_RST_LOW();
    HAL_Delay(50);
    LCD_RST_HIGH();
    HAL_Delay(150);

    WriteReg(0x11); // SLPOUT
    HAL_Delay(120);

    // RGB565
    WriteReg(0x3A);
    uint8_t colmod = 0x55;
    WriteData(&colmod, 1);

    // BGR + portrait
    WriteReg(0x36);
    uint8_t madctl = 0x00;
    WriteData(&madctl, 1);

    WriteReg(0x21); // INVON
    HAL_Delay(10);

    WriteReg(0x29); // DISPON
    HAL_Delay(20);
}

void setPWM_PE9(uint16_t value)
{
    TIM1->CCR1 = value;
}

void Clear(uint16_t color)
{
	SetWindowColor(0, 0, LCD_WIDTH, LCD_HEIGHT, color);
}

void SetPixel(uint16_t x, uint16_t y, uint16_t color)
{
    if (x > LCD_WIDTH || y > LCD_HEIGHT)
    {
        return;
    }
    SetCursor(x, y, x, y);
    WriteData((uint8_t*)&color, 2);
}

void DrawImage(const uint16_t* image, int16_t xStart, int16_t yStart, int16_t xEnd, int16_t yEnd)
{

    static uint16_t lineBuf[LCD_WIDTH];

    if ((xStart + xEnd) > LCD_WIDTH || (yStart + yEnd) > LCD_HEIGHT)
        return;

    SetCursor(xStart, yStart, xStart + xEnd - 1, yStart + yEnd - 1);

    LCD_DC_HIGH();
    LCD_CS_LOW();

    for (uint32_t row = 0u; row < yEnd; row++)
    {
        for (uint32_t col = 0u; col < xEnd; col++)
        {
            lineBuf[col] = image[row * xEnd + col];
        }

        HAL_SPI_Transmit(&hspi1, (uint8_t*)lineBuf, xEnd * 2, HAL_MAX_DELAY);
    }

    LCD_CS_HIGH();
}

void DrawString(int16_t xStart, int16_t yStart, const char* pString, Font_t* font, int16_t colorBackground, int16_t colorForeround, uint8_t useBackgroundColor)
{
    int16_t xPoint = xStart;
    int16_t yPoint = yStart;

    // if (xStart > LCD_WIDTH || yStart > LCD_HEIGHT) {
    //     Debug("Paint_DrawString_EN Input exceeds the normal display range\r\n");
    //     return;
    // }

    while (*pString != '\0')
    {
        // if X direction filled , reposition to(xStart,yPoint),yPoint is Y direction plus the Height of the character
        if ((xPoint + font->Width) > LCD_WIDTH)
        {
            xPoint = xStart;
            yPoint += font->Height;
        }

        // If the Y direction is full, reposition to(xStart, yStart)
        if ((yPoint + font->Height) > LCD_HEIGHT)
        {
            xPoint = xStart;
            yPoint = yStart;
        }
        DrawChar(xPoint, yPoint, *pString, font, colorBackground, colorForeround, useBackgroundColor);

        // The next character of the address
        pString++;

        // The next word of the abscissa increases the font of the broadband
        xPoint += font->Width;
    }
}

void DrawChar(int16_t xPoint, int16_t yPoint, const char asciiChar, Font_t* font, int16_t colorBackground, int16_t colorForeround, uint8_t useBackgroundColor)
{
    int16_t page, Column;

    // if (xPoint > LCD_WIDTH || yPoint > LCD_HEIGHT) {
    //     Debug("Paint_DrawChar Input exceeds the normal display range\r\n");
    //     return;
    // }
    uint32_t Char_Offset     = (asciiChar - ' ') * font->Height * (font->Width / 8 + (font->Width % 8 ? 1 : 0));
    const unsigned char* ptr = &font->table[Char_Offset];

    for (page = 0; page < font->Height; page++)
    {
        for (Column = 0; Column < font->Width; Column++)
        {

            // To determine whether the font background color and screen background color is consistent
            if (WHITE == colorBackground)
            { // this process is to speed up the scan
                if ((*ptr) & (0x80 >> (Column % 8)))
                    SetPixel(xPoint + Column, yPoint + page, colorForeround);
            }
            else
            {
                if ((*ptr) & (0x80 >> (Column % 8)))
                {
                    SetPixel(xPoint + Column, yPoint + page, colorForeround);
                }
                else if (useBackgroundColor != FALSE)
                {
                    SetPixel(xPoint + Column, yPoint + page, colorBackground);
                }
            }
            // One pixel is 8 bits
            if (Column % 8 == 7)
            {
                ptr++;
            }
        } /* Write a line */
        if (font->Width % 8 != 0)
        {
            ptr++;
        }
    } /* Write all */
}
