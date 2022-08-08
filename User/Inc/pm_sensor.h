/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    pm_sensor.h
 * @brief   This file contains all the function prototypes for
 *          the pm_sensor.c file
 ******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PM_SENSOR_H__
#define __PM_SENSOR_H__

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx.h"
#include "dma.h"
#include "usart.h"

    /* USER CODE BEGIN Includes */

    /* USER CODE END Includes */

    /* USER CODE BEGIN Private defines */
#define SENSOR_ADDRESS huart1
#define HEAD 0x68
#define LENGTH 0x01
#define CMD_READ_DATA 0x04
#define CMD_START_TRANS_AUTO 0x40
#define CMD_STOP_TRANS_AUTO 0x20
#define CS_READ_DATA 0x93
#define CS_START_TRANS_AUTO 0x47
#define CS_STOP_TRANS_AUTO 0x77
#define PM_FAIL 0x96

    /* USER CODE END Private defines */

    /* USER CODE BEGIN Prototypes */
    void PM_Read_Data(void);           //读取微粒测量结果
    void PM_Stop_Transmit_Auto(void);  //传感器停止自动发送
    void PM_Start_Transmit_Auto(void); //传感器开启自动发送
    /* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif
