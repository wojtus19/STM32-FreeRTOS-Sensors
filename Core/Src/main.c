/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BH1750/LightSensor.h"
#include "BME280/bme.h"
#include "FreeRTOS.h"
#include "LCD_Screen/LCD_Screen.h"
#include "VL53L0X/DistanceSensor.h"
#include "event_groups.h"
#include "i2c_manager.h"
#include "logger.h"
#include "shrek.h"
#include "task.h"
#include "utils.h"
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PRIO_INIT 5
#define PRIO_I2C 4
#define PRIO_LCD 3
#define PRIO_DISTANCE_SENSOR 2
#define PRIO_LIGHT_SENSOR 2
#define PRIO_BME 2
#define PRIO_LOGGER 1
#define PRIO_DRAW_IMAGE 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s3;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
TaskHandle_t logger_task_handle;
TaskHandle_t LCD_screen_task_handle;
TaskHandle_t I2C_ManagerTaskHandle;
TaskHandle_t distance_measure_task_handle;
TaskHandle_t bme_task_handle;
TaskHandle_t init_task_handle;
// TaskHandle_t draw_image_task_handle;
TaskHandle_t light_sensor_task_handle;

EventGroupHandle_t i2c_event_group;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
static void DistanceMeasureTask(void* parameters);
static void BMETask(void* parameters);
static void InitTask(void* parameters);
// static void DrawImageTask(void* parameter);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

    /* USER CODE BEGIN 1 */
    BaseType_t status;
    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_I2C1_Init();
    MX_I2S3_Init();
    MX_SPI1_Init();
    MX_USB_HOST_Init();
    MX_USART2_UART_Init();
    MX_TIM1_Init();
    /* USER CODE BEGIN 2 */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

    i2c_event_group = xEventGroupCreate();
    configASSERT(i2c_event_group);

    status = xTaskCreate(InitTask, "Init-Task", 200, "", PRIO_INIT, &init_task_handle);

    configASSERT(pdPASS == status);

    /* W/A: Set Priorities NVIC after CubeMX Code Generation */
    HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
    HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);

    vTaskStartScheduler();

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */
        MX_USB_HOST_Process();

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

    /** Configure the main internal regulator output voltage
     */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM       = 8;
    RCC_OscInitStruct.PLL.PLLN       = 336;
    RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ       = 7;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

    /* USER CODE BEGIN I2C1_Init 0 */

    /* USER CODE END I2C1_Init 0 */

    /* USER CODE BEGIN I2C1_Init 1 */

    /* USER CODE END I2C1_Init 1 */
    hi2c1.Instance             = I2C1;
    hi2c1.Init.ClockSpeed      = 100000;
    hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1     = 0;
    hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2     = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN I2C1_Init 2 */

    /* USER CODE END I2C1_Init 2 */
}

/**
 * @brief I2S3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2S3_Init(void)
{

    /* USER CODE BEGIN I2S3_Init 0 */

    /* USER CODE END I2S3_Init 0 */

    /* USER CODE BEGIN I2S3_Init 1 */

    /* USER CODE END I2S3_Init 1 */
    hi2s3.Instance            = SPI3;
    hi2s3.Init.Mode           = I2S_MODE_MASTER_TX;
    hi2s3.Init.Standard       = I2S_STANDARD_PHILIPS;
    hi2s3.Init.DataFormat     = I2S_DATAFORMAT_16B;
    hi2s3.Init.MCLKOutput     = I2S_MCLKOUTPUT_ENABLE;
    hi2s3.Init.AudioFreq      = I2S_AUDIOFREQ_96K;
    hi2s3.Init.CPOL           = I2S_CPOL_LOW;
    hi2s3.Init.ClockSource    = I2S_CLOCK_PLL;
    hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
    if (HAL_I2S_Init(&hi2s3) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN I2S3_Init 2 */

    /* USER CODE END I2S3_Init 2 */
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void)
{

    /* USER CODE BEGIN SPI1_Init 0 */
    __HAL_RCC_SPI1_CLK_ENABLE();  // włącz zegar SPI1
    __HAL_RCC_GPIOA_CLK_ENABLE(); // SPI1 używa pinów z portu A

    GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    GPIO_InitStruct.Pin              = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7; // PA5=SCK, PA6=MISO, PA7=MOSI
    GPIO_InitStruct.Mode             = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull             = GPIO_NOPULL;
    GPIO_InitStruct.Speed            = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate        = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    /* USER CODE END SPI1_Init 0 */

    /* USER CODE BEGIN SPI1_Init 1 */

    /* USER CODE END SPI1_Init 1 */
    /* SPI1 parameter configuration*/
    hspi1.Instance               = SPI1;
    hspi1.Init.Mode              = SPI_MODE_MASTER;
    hspi1.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity       = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase          = SPI_PHASE_1EDGE;
    hspi1.Init.NSS               = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
    hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial     = 10;
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN SPI1_Init 2 */

    /* USER CODE END SPI1_Init 2 */
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{

    /* USER CODE BEGIN TIM1_Init 0 */
    TIM_OC_InitTypeDef sConfigOC = { 0 };

    /* USER CODE END TIM1_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
    TIM_MasterConfigTypeDef sMasterConfig     = { 0 };

    /* USER CODE BEGIN TIM1_Init 1 */

    /* USER CODE END TIM1_Init 1 */
    htim1.Instance               = TIM1;
    htim1.Init.Prescaler         = 84 - 1;
    htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim1.Init.Period            = 255 - 1;
    htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM1_Init 2 */
    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.Pulse      = 0; // Początkowe wypełnienie = 0
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE END TIM1_Init 2 */
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

    /* USER CODE BEGIN USART2_Init 0 */

    /* USER CODE END USART2_Init 0 */

    /* USER CODE BEGIN USART2_Init 1 */

    /* USER CODE END USART2_Init 1 */
    huart2.Instance          = USART2;
    huart2.Init.BaudRate     = 115200;
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.Mode         = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN USART2_Init 2 */

    /* USER CODE END USART2_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

    /* DMA controller clock enable */
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    /* DMA interrupt init */
    /* DMA1_Stream6_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
    /* DMA2_Stream3_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    /* USER CODE BEGIN MX_GPIO_Init_1 */

    /* USER CODE END MX_GPIO_Init_1 */

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2 | CS_I2C_SPI_Pin | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOD, LD4_Pin | LD3_Pin | LD5_Pin | LD6_Pin | Audio_RST_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pins : PE2 CS_I2C_SPI_Pin PE4 PE5
                             PE6 */
    GPIO_InitStruct.Pin   = GPIO_PIN_2 | CS_I2C_SPI_Pin | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /*Configure GPIO pins : OTG_FS_PowerSwitchOn_Pin PC1 */
    GPIO_InitStruct.Pin   = OTG_FS_PowerSwitchOn_Pin | GPIO_PIN_1;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pin : PDM_OUT_Pin */
    GPIO_InitStruct.Pin       = PDM_OUT_Pin;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : B1_Pin */
    GPIO_InitStruct.Pin  = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : BOOT1_Pin */
    GPIO_InitStruct.Pin  = BOOT1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : PE11 */
    GPIO_InitStruct.Pin  = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /*Configure GPIO pin : PE12 */
    GPIO_InitStruct.Pin  = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /*Configure GPIO pin : CLK_IN_Pin */
    GPIO_InitStruct.Pin       = CLK_IN_Pin;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                             Audio_RST_Pin */
    GPIO_InitStruct.Pin   = LD4_Pin | LD3_Pin | LD5_Pin | LD6_Pin | Audio_RST_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
    GPIO_InitStruct.Pin  = OTG_FS_OverCurrent_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : MEMS_INT2_Pin */
    GPIO_InitStruct.Pin  = MEMS_INT2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

    /* USER CODE BEGIN MX_GPIO_Init_2 */
    GPIO_InitStruct.Pin       = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

static void DistanceMeasureTask(void* parameters)
{
    /* 5000ms delay */
    const TickType_t xDelay = 3000 / portTICK_PERIOD_MS;
    uint16_t distance;
    char distanceStr[20];
    for (;;)
    {
        /* Toggle Blue LED */
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
        distance = readRangeSingleMillimeters();
        LogPrintf(LOG_INFO, "Distance: %d\n", distance);
        itoa(distance, distanceStr, 10);

        LCD_FillRect(0, 0, LCD_WIDTH, 60, BLACK);
        LCD_DrawString(10, 20, distanceStr, &Font20, GREEN, BLACK);
        vTaskDelay(xDelay);
    }
}

static void BMETask(void* parameters)
{
    /* 5000ms delay */
    const TickType_t xDelay = 2000 / portTICK_PERIOD_MS;
    BME280_Data_t bme_data;
    //_Accum temperature, pressure, humidity;
    char tempStr[8], pressureStr[8], humidityStr[8];
    for (;;)
    {
        bme_data = BME280_ReadData();
        // bme_data.temperature = BME280_ReadTemperature();
        // bme_data.pressure    = BME280_ReadPressure();
        // bme_data.humidity    = BME280_ReadHumidity();

        LCD_FillRect(0, 60, LCD_WIDTH, 180, BLACK);

        snprintf(tempStr, sizeof(tempStr), "%.2f", bme_data.temperature);
        // AccumToString(bme_data.temperature, tempStr, 2);
        LCD_DrawString(10, 60, tempStr, &Font20, CYAN, BLACK);

        snprintf(pressureStr, sizeof(pressureStr), "%.2f", bme_data.pressure);
        // AccumToString(bme_data.pressure, pressureStr, 2);
        LCD_DrawString(10, 100, pressureStr, &Font20, MAGENTA, BLACK);

        snprintf(humidityStr, sizeof(humidityStr), "%.2f", bme_data.humidity);
        // AccumToString(bme_data.humidity, humidityStr, 2);
        LCD_DrawString(10, 140, humidityStr, &Font20, LGRAYBLUE, BLACK);

        vTaskDelay(xDelay);
    }
}

static void LightSensorTask(void* parameters)
{
    /* 5000ms delay */
    const TickType_t xDelay = 2000 / portTICK_PERIOD_MS;
    float lux               = 0u;
    char luxStr[8]          = { 0 };
    for (;;)
    {

        lux = BH1750_ReadLux();
        snprintf(luxStr, sizeof(luxStr), "%.2f", lux);

        LCD_FillRect(0, 180, LCD_WIDTH, LCD_HEIGHT, BLACK);
        LCD_DrawString(10, 180, luxStr, &Font20, YELLOW, BLACK);

        vTaskDelay(xDelay);
    }
}

static void InitTask(void* parameter)
{
    BaseType_t status;

    I2C_ManagerInit(&hi2c1);

    status = xTaskCreate(I2C_ManagerTask, "I2C-Manager", 200, "", PRIO_I2C, &I2C_ManagerTaskHandle);
    configASSERT(pdPASS == status);

    LoggerInit();

    status = Init_VL53L0X(TRUE); // Initialize Distance sensor
    configASSERT(pdPASS == status);

    status = BME280_Init();
    configASSERT(STATUS_OK == status);
    BH1750_Init();

    LCD_ScreenInit();
    LCD_ChangeBrightness(150);
    LCD_FillScreen(BLACK);
    // LCD_DrawImage(shrek_img, 0, 0, LCD_WIDTH, LCD_HEIGHT);

    LogPrintf(LOG_INFO, "Sensors initialized\n", xTaskGetTickCount() * portTICK_PERIOD_MS);

    status = xTaskCreate(DistanceMeasureTask, "Distance", 200, "", PRIO_DISTANCE_SENSOR, &distance_measure_task_handle);
    configASSERT(pdPASS == status);

    status = xTaskCreate(BMETask, "BME280", 768, "", PRIO_BME, &bme_task_handle);
    configASSERT(pdPASS == status);

    status = xTaskCreate(LCDScreenTask, "LCDScreen", 200, "", PRIO_LCD, &LCD_screen_task_handle);
    configASSERT(pdPASS == status);

    status = xTaskCreate(LoggerTask, "Logger", 1024, NULL, PRIO_LOGGER, &logger_task_handle);
    configASSERT(pdPASS == status);

    status = xTaskCreate(LightSensorTask, "Light", 512, NULL, PRIO_LIGHT_SENSOR, &light_sensor_task_handle);
    configASSERT(pdPASS == status);

    // status = xTaskCreate(DrawImageTask, "Draw-Image", 512, NULL, PRIO_DRAW_IMAGE, &draw_image_task_handle);
    // configASSERT(pdPASS == status);

    vTaskDelete(NULL);
}

// static void DrawImageTask(void* parameter)
//{
//     const TickType_t xDelay = 5000 / portTICK_PERIOD_MS;
//     for (;;)
//     {
//         HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
//         LCD_DrawImage(shrek_img, 0, 0, LCD_WIDTH, LCD_HEIGHT);
//         vTaskDelay(xDelay);
//     }
// }
/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM6 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    /* USER CODE BEGIN Callback 0 */

    /* USER CODE END Callback 0 */
    if (htim->Instance == TIM6)
    {
        HAL_IncTick();
    }
    /* USER CODE BEGIN Callback 1 */

    /* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
