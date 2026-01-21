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

static void WriteReg(uint8_t data);
static void WriteData_Word(uint16_t data);
static void WriteReg(uint8_t data);

static void WriteData_Word(uint16_t data)
{
    uint8_t rx = 0u;
    uint8_t i  = (data >> 8) & 0xff;
    HAL_GPIO_WritePin(CS_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DC_PIN, GPIO_PIN_SET);
    HAL_SPI_TransmitReceive(&hspi1, &i, &rx, 1, HAL_MAX_DELAY);
    HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&data, &rx, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(CS_PIN, GPIO_PIN_SET);
}

static void WriteData_Byte(uint8_t data)
{
    HAL_GPIO_WritePin(CS_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DC_PIN, GPIO_PIN_SET);
    HAL_SPI_Transmit(&hspi1, (uint8_t*)&data, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(CS_PIN, GPIO_PIN_SET);
}

static void WriteReg(uint8_t data)
{
    HAL_GPIO_WritePin(CS_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DC_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, (uint8_t*)&data, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(CS_PIN, GPIO_PIN_SET);
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

void SetWindowcolor(uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd, uint16_t color)
{
    SetCursor(xStart, yStart, xEnd, yEnd);
    WriteData_Word(color);
}

void ClearWindow(uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd, uint16_t color)
{
    uint16_t i, j;
    SetCursor(xStart, yStart, xEnd, yEnd);
    for (i = yStart; i < yEnd; i++)
    {
        for (j = xStart; j < xEnd; j++)
        {
            WriteData_Word(color);
        }
    }
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
    ScreenReset();
    WriteReg(0x36);
    WriteData_Byte(0x00);
    WriteReg(0x3A);
    WriteData_Byte(0x05);
    WriteReg(0xB2);
    WriteData_Byte(0x0B);
    WriteData_Byte(0x0B);
    WriteData_Byte(0x00);
    WriteData_Byte(0x33);
    WriteData_Byte(0x35);

    WriteReg(0xB7);
    WriteData_Byte(0x11);

    WriteReg(0xBB);
    WriteData_Byte(0x35);

    WriteReg(0xC0);
    WriteData_Byte(0x2C);

    WriteReg(0xC2);
    WriteData_Byte(0x01);

    WriteReg(0xC3);
    WriteData_Byte(0x0D);

    WriteReg(0xC4);
    WriteData_Byte(0x20);

    WriteReg(0xC6);
    WriteData_Byte(0x13);

    WriteReg(0xD0);
    WriteData_Byte(0xA4);
    WriteData_Byte(0xA1);

    WriteReg(0xD6);
    WriteData_Byte(0xA1);

    WriteReg(0xE0);
    WriteData_Byte(0xF0);
    WriteData_Byte(0x06);
    WriteData_Byte(0x0B);
    WriteData_Byte(0x0A);
    WriteData_Byte(0x09);
    WriteData_Byte(0x26);
    WriteData_Byte(0x29);
    WriteData_Byte(0x33);
    WriteData_Byte(0x41);
    WriteData_Byte(0x18);
    WriteData_Byte(0x16);
    WriteData_Byte(0x15);
    WriteData_Byte(0x29);
    WriteData_Byte(0x2D);

    WriteReg(0xE1);
    WriteData_Byte(0xF0);
    WriteData_Byte(0x04);
    WriteData_Byte(0x08);
    WriteData_Byte(0x08);
    WriteData_Byte(0x07);
    WriteData_Byte(0x03);
    WriteData_Byte(0x28);
    WriteData_Byte(0x32);
    WriteData_Byte(0x40);
    WriteData_Byte(0x3B);
    WriteData_Byte(0x19);
    WriteData_Byte(0x18);
    WriteData_Byte(0x2A);
    WriteData_Byte(0x2E);

    WriteReg(0xE4);
    WriteData_Byte(0x25);
    WriteData_Byte(0x00);
    WriteData_Byte(0x00);

    WriteReg(0x21);
    WriteReg(0x11);
    HAL_Delay(120);
    WriteReg(0x29);
}

void setPWM_PE9(uint16_t value)
{
    TIM1->CCR1 = value;
}

void Clear(uint16_t color)
{
    uint16_t i, j;
    SetCursor(0, 0, LCD_WIDTH, LCD_HEIGHT);
    for (i = 0; i < LCD_WIDTH; i++)
    {
        for (j = 0; j < LCD_HEIGHT; j++)
        {
            WriteData_Word(color);
        }
    }
}

void SetPixel(uint16_t x, uint16_t y, uint16_t color)
{
    if (x > LCD_WIDTH || y > LCD_HEIGHT)
    {
        return;
    }
    SetCursor(x, y, x, y);
    WriteData_Word(color);
}

void DrawImage(const uint8_t* image, int16_t xStart, int16_t yStart, int16_t W_Image, int16_t H_Image)
{
    int i, j;
    for (j = 0; j < H_Image; j++)
    {
        for (i = 0; i < W_Image; i++)
        {
            SetPixel(xStart + i, yStart + j, image[j * W_Image * 2 + i * 2 + 1] << 8 | image[j * W_Image * 2 + i * 2]);
        }
    }
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
