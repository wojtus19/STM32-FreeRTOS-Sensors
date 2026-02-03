# FreeRTOS on STM32F407G-DISC1 with Sensors and Display

This project demonstrates the use of FreeRTOS on the STM32F407G-DISC1 board, integrating multiple I2C sensors, UART logging, and SPI display output.

## Features
- **I2C Sensors**: 
  - BME280 (temperature, humidity, pressure) in a dedicated task.
  - VL53L0X (distance) in a dedicated task.
  - BH1750 (light intensity) in a dedicated task.
  - Each sensor task sends I2C requests to a central `I2C_Manager` task for coordinated bus access.
- **Logger**: A dedicated task for logging data via UART with DMA.
- **Display**: A dedicated task for displaying sensor data on an ST7789V2 LCD via SPI with DMA.
- **Diagnostics**: A dedicated task for measuring CPU Usage, Runtime stats and highwater marks

## Hardware Requirements
- STM32F407G-DISC1 development board.
- BME280, VL53L0X, and BH1750 sensors connected via I2C.
- ST7789V2 LCD connected via SPI.
- UART connection for logging (e.g., to a serial console).
