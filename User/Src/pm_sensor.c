/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    pm_sensor.c
 * @brief   This file provides code for configuration
 *          of PM Sensor. Model:HPMA115S0
 *******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "pm_sensor.h"

uint8_t PM_READ_DATA_COMMAND[4] = {HEAD, LENGTH, CMD_READ_DATA, CS_READ_DATA};
uint8_t PM_START_TRANS_AUTO_COMMAND[4] = {HEAD, LENGTH, CMD_READ_DATA, CS_READ_DATA};
uint8_t PM_STOP_TRANS_AUTO_COMMAND[4] = {HEAD, LENGTH, CMD_STOP_TRANS_AUTO, CS_STOP_TRANS_AUTO};
uint8_t PM_RX_DATA_BUFFER[8];
uint8_t PM_RX_RESPOND_BUFFER[2];
uint16_t PM_2_5_DATA;

void PM_Read_Data(void) //读取微粒测量结果
{
    uint8_t pm_25_bit1 = PM_RX_DATA_BUFFER[3];
    uint8_t pm_25_bit0 = PM_RX_DATA_BUFFER[4];
    HAL_UART_Transmit(&SENSOR_PORT, PM_READ_DATA_COMMAND, 4, 1000);
    HAL_UART_Receive_DMA(&SENSOR_PORT, PM_RX_DATA_BUFFER, 8);
    PM_2_5_DATA = pm_25_bit1 * 256 + pm_25_bit0;
}

void PM_Stop_Transmit_Auto(void) //传感器停止自动发送
{
    HAL_UART_Transmit(&SENSOR_PORT, PM_STOP_TRANS_AUTO_COMMAND, 4, 1000);
    HAL_UART_Receive_DMA(&SENSOR_PORT, PM_RX_RESPOND_BUFFER, 2);
}

void PM_Start_Transmit_Auto(void) //传感器开启自动发送
{
    HAL_UART_Transmit(&SENSOR_PORT, PM_START_TRANS_AUTO_COMMAND, 4, 1000);
    HAL_UART_Receive_DMA(&SENSOR_PORT, PM_RX_RESPOND_BUFFER, 2);
}
