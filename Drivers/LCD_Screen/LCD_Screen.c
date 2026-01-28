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
#include "FreeRTOS.h"
#include "logger.h"
#include "memory_utils.h"
#include "semphr.h"
#include "stream_buffer.h"
#include <string.h>

#define DMA_COLOR_CHUNK_PIXELS 256
#define DMA_COLOR_CHUNK_BYTES (DMA_COLOR_CHUNK_PIXELS * 2)
#define LCD_DMA_CHUNK_SIZE 4096
#define LCD_QUEUE_DEPTH 2 // double buffering

#define LCD_CS_LOW() HAL_GPIO_WritePin(CS_PIN, GPIO_PIN_RESET)
#define LCD_CS_HIGH() HAL_GPIO_WritePin(CS_PIN, GPIO_PIN_SET)

#define LCD_DC_LOW() HAL_GPIO_WritePin(DC_PIN, GPIO_PIN_RESET)
#define LCD_DC_HIGH() HAL_GPIO_WritePin(DC_PIN, GPIO_PIN_SET)

#define LCD_RST_LOW() HAL_GPIO_WritePin(RST_PIN, GPIO_PIN_RESET)
#define LCD_RST_HIGH() HAL_GPIO_WritePin(RST_PIN, GPIO_PIN_SET)

static void WriteData(uint8_t* data, uint32_t size);
static void WriteData_Byte(uint8_t data);
static void WriteReg(uint8_t data);
static void LCDTaskInit(void);
static void FillScreenDMA(uint16_t color, uint32_t pixel_count);
static void FillImageDMA(const uint16_t* pixels, uint32_t pixel_count);
static void DrawString(int16_t xStart, int16_t yStart, const char* pString, Font_t* font, int16_t colorBackground, int16_t colorForeground);
static void DrawChar(int16_t xPoint, int16_t yPoint, const char ascii_char, Font_t* font, int16_t colorBackground, int16_t colorForeground);

SemaphoreHandle_t spiDmaDoneSem;
extern TaskHandle_t LCD_screen_task_handle;
static QueueHandle_t lcdQueue;

typedef enum lcd_cmd_t
{
    LCD_CMD_FILL_SCREEN,
    LCD_CMD_FILL_RECT,
    LCD_CMD_DRAW_IMAGE,
    LCD_CMD_DRAW_TEXT
} lcd_cmd_t;

typedef struct lcd_frame_t
{
    lcd_cmd_t type;
    union
    {
        struct
        {
            uint16_t color;
        } fill_screen;
        struct
        {
            uint16_t xStart, yStart;
            uint16_t xEnd, yEnd;
            uint16_t color;
        } fill_rect;

        struct
        {
            uint16_t xStart, yStart;
            uint16_t xEnd, yEnd;
            const uint16_t* pixels;
        } image;
        struct
        {
            uint16_t xStart, yStart;
            const char* text;
            uint16_t color;
            uint16_t bg;
            Font_t* font;
        } text;
    } data;

} lcd_frame_t;

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

void SetWindow(uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd)
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

void ScreenReset()
{
    HAL_GPIO_WritePin(CS_PIN, GPIO_PIN_RESET);
    HAL_Delay(20);
    HAL_GPIO_WritePin(RST_PIN, GPIO_PIN_RESET);
    HAL_Delay(20);
    HAL_GPIO_WritePin(RST_PIN, GPIO_PIN_SET);
    HAL_Delay(20);
}

void LCD_ScreenInit()
{
    LCDTaskInit();
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

void LCD_ChangeBrightness(uint16_t value)
{
    TIM1->CCR1 = value;
}

void LCDScreenTask(void* argument)
{
    lcd_frame_t frame;
    uint32_t pixel_count = 0u;
    for (;;)
    {
        xQueueReceive(lcdQueue, &frame, portMAX_DELAY);
        switch (frame.type)
        {
        case LCD_CMD_FILL_SCREEN:
            SetWindow(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1);
            pixel_count = LCD_WIDTH * LCD_HEIGHT;
            FillScreenDMA(frame.data.fill_screen.color, pixel_count);
            break;

        case LCD_CMD_FILL_RECT:
            SetWindow(frame.data.fill_rect.xStart, frame.data.fill_rect.yStart, frame.data.fill_rect.xEnd - 1, frame.data.fill_rect.yEnd - 1);
            pixel_count = (frame.data.fill_rect.xEnd - frame.data.fill_rect.xStart) * (frame.data.fill_rect.yEnd - frame.data.fill_rect.yStart);
            FillScreenDMA(frame.data.fill_rect.color, pixel_count);
            break;

        case LCD_CMD_DRAW_IMAGE:
            SetWindow(frame.data.image.xStart, frame.data.image.yStart, frame.data.image.xEnd - 1, frame.data.image.yEnd - 1);
            pixel_count = (frame.data.fill_rect.xEnd - frame.data.fill_rect.xStart) * (frame.data.fill_rect.yEnd - frame.data.fill_rect.yStart);
            FillImageDMA(frame.data.image.pixels, pixel_count);
            break;

        case LCD_CMD_DRAW_TEXT:
            DrawString(frame.data.text.xStart, frame.data.text.yStart, frame.data.text.text, frame.data.text.font, frame.data.text.bg, frame.data.text.color);
            break;
        }
    }
}

static void DrawString(int16_t xStart, int16_t yStart, const char* pString, Font_t* font, int16_t colorBackground, int16_t colorForeground)
{
    int16_t xPoint = xStart;
    int16_t yPoint = yStart;

    while (*pString != '\0')
    {
        // if X direction filled , reposition to(xStart,yPoint),yPoint is Y direction plus the Height of the character
        if ((xPoint + font->width) > LCD_WIDTH)
        {
            xPoint = xStart;
            yPoint += font->height;
        }

        // If the Y direction is full, reposition to(xStart, yStart)
        if ((yPoint + font->height) > LCD_HEIGHT)
        {
            xPoint = xStart;
            yPoint = yStart;
        }
        DrawChar(xPoint, yPoint, *pString, font, colorBackground, colorForeground);

        // The next character of the address
        pString++;

        // The next word of the abscissa increases the font of the broadband
        xPoint += font->width;
    }
}

static void DrawChar(int16_t xPoint, int16_t yPoint, const char ascii_char, Font_t* font, int16_t colorBackground, int16_t colorForeground)
{
    uint32_t pixel_count = font->width * font->height;
    uint16_t* buffer     = pvPortMalloc(pixel_count * sizeof(uint16_t));
    configASSERT(buffer);
    int16_t page, column;

    uint32_t char_offset     = (ascii_char - ' ') * font->height * (font->width / 8 + (font->width % 8 ? 1 : 0));
    const unsigned char* ptr = &font->table[char_offset];

    for (page = 0; page < font->height; page++)
    {
        for (column = 0; column < font->width; column++)
        {

            // To determine whether the font background color and screen background color is consistent
            if (WHITE == colorBackground)
            { // this process is to speed up the scan
                if ((*ptr) & (0x80 >> (column % 8)))
                {
                    buffer[page * font->width + column] = colorForeground;
                }
            }
            else
            {
                if ((*ptr) & (0x80 >> (column % 8)))
                {
                    buffer[page * font->width + column] = colorForeground;
                }
                else // if (useBackgroundColor != FALSE)
                {
                    buffer[page * font->width + column] = colorBackground;
                }
            }
            // One pixel is 8 bits
            if (column % 8 == 7)
            {
                ptr++;
            }
        } /* Write a line */
        if (font->width % 8 != 0)
        {
            ptr++;
        }
    }
    SetWindow(xPoint, yPoint, xPoint + font->width - 1, yPoint + font->height - 1);
    FillImageDMA(buffer, pixel_count);
    vPortFree(buffer);
}

void LCD_FillScreen(uint16_t color)
{
    lcd_frame_t frame;

    frame.type                   = LCD_CMD_FILL_SCREEN;
    frame.data.fill_screen.color = color;

    if (xQueueSend(lcdQueue, &frame, portMAX_DELAY) != pdPASS)
    {
        LogPrintf("LCD Queue timeout");
    }
}

void LCD_DrawImage(const uint16_t* image, int16_t xStart, int16_t yStart, int16_t xEnd, int16_t yEnd)
{
    lcd_frame_t frame;

    frame.type              = LCD_CMD_DRAW_IMAGE;
    frame.data.image.pixels = image;
    frame.data.image.xStart = xStart;
    frame.data.image.yStart = yStart;
    frame.data.image.xEnd   = xEnd;
    frame.data.image.yEnd   = yEnd;

    if (xQueueSend(lcdQueue, &frame, portMAX_DELAY) != pdPASS)
    {
        LogPrintf("LCD Queue timeout");
    }
}

void LCD_FillRect(uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd, uint16_t color)
{
    lcd_frame_t frame;

    frame.type                  = LCD_CMD_FILL_RECT;
    frame.data.fill_rect.color  = color;
    frame.data.fill_rect.xStart = xStart;
    frame.data.fill_rect.yStart = yStart;
    frame.data.fill_rect.xEnd   = xEnd;
    frame.data.fill_rect.yEnd   = yEnd;

    if (xQueueSend(lcdQueue, &frame, portMAX_DELAY) != pdPASS)
    {
        LogPrintf("LCD Queue timeout");
    }
}

void LCD_DrawString(int16_t xStart, int16_t yStart, const char* pString, Font_t* font, int16_t colorForeground, int16_t colorBackground)
{
    lcd_frame_t frame;

    frame.type             = LCD_CMD_DRAW_TEXT;
    frame.data.text.xStart = xStart;
    frame.data.text.yStart = yStart;
    frame.data.text.text   = pString;
    frame.data.text.color  = colorForeground;
    frame.data.text.bg     = colorBackground;
    frame.data.text.font   = font;

    if (xQueueSend(lcdQueue, &frame, portMAX_DELAY) != pdPASS)
    {
        LogPrintf("LCD Queue timeout");
    }
}

static void LCDTaskInit(void)
{
    spiDmaDoneSem = xSemaphoreCreateBinary();

    lcdQueue = xQueueCreate(LCD_QUEUE_DEPTH, sizeof(lcd_frame_t));
}

static void FillImageDMA(const uint16_t* image, uint32_t pixel_count)
{
    static uint16_t image_buf[DMA_COLOR_CHUNK_PIXELS];

    LCD_CS_LOW();
    LCD_DC_HIGH();

    uint32_t remaining = pixel_count;
    uint32_t cpyIdx    = 0u;
    while (remaining > 0)
    {
        uint32_t pixels = (remaining > DMA_COLOR_CHUNK_PIXELS) ? DMA_COLOR_CHUNK_PIXELS : remaining;

        memcpy(image_buf, &image[cpyIdx], pixels * 2);

        HAL_SPI_Transmit_DMA(&hspi1, (uint8_t*)image_buf, pixels * 2);

        xSemaphoreTake(spiDmaDoneSem, portMAX_DELAY);

        remaining -= pixels;
        cpyIdx += pixels;
    }
    LCD_CS_HIGH();
}

static void FillScreenDMA(uint16_t color, uint32_t pixel_count)
{
    static uint16_t color_buf[DMA_COLOR_CHUNK_PIXELS];
    for (uint32_t idx = 0; idx < DMA_COLOR_CHUNK_PIXELS; idx++)
    {
        color_buf[idx] = color;
    }

    LCD_CS_LOW();
    LCD_DC_HIGH();

    uint32_t remaining = pixel_count;
    while (remaining > 0)
    {
        uint32_t pixels = (remaining > DMA_COLOR_CHUNK_PIXELS) ? DMA_COLOR_CHUNK_PIXELS : remaining;

        HAL_SPI_Transmit_DMA(&hspi1, (uint8_t*)color_buf, pixels * 2);

        xSemaphoreTake(spiDmaDoneSem, portMAX_DELAY);

        remaining -= pixels;
    }
    LCD_CS_HIGH();
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef* hspi)
{
    if (hspi == &hspi1)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(spiDmaDoneSem, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
