/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : Radio.h
 * @brief          : Header for Radio.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 Sang Tan Truong.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RADIO_H_
#define __RADIO_H_

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>

#include "main.h"  //To use printf function
#include "stm32f2xx.h"

/* Private includes ----------------------------------------------------------*/
#include "MY_NRF24.h"
#include "Oled.h"
#include "uartRingBufDMA.h"
/* Private defines -----------------------------------------------------------*/
#define RADIO_CIRCULAR_BUFFER_SIZE 128

/* Exported types ------------------------------------------------------------*/
typedef struct RadioInit_t {
    NRF24_Init_t nrfInit;
    RingHandler_t *Serial;
    bool enableDebug;
    RingHandler_t *Debug;
    OLEDDisplay_t *Display;
    uint16_t *ADCValue;
} RadioInit_t;

typedef struct VariRes_Value_t {
    uint16_t Adc1;
    uint16_t Adc2;
    uint16_t Adc3;
    uint16_t Adc4;
} VariRes_Value_t;

typedef enum RadioStatus_t {
    RADIO_OK = 0U,
    RADIO_EMPTY = 1U,
    RADIO_FULL = 2U,
} RadioStatus_t;

typedef struct RadioHandler_t {
    RadioInit_t Init;
    RadioStatus_t Status;
    RingHandler_t *Serial;
    RingHandler_t *Debug;
    bool enableDebug;
    OLEDDisplay_t *Display;
    VariRes_Value_t PWMValue;
    VariRes_Value_t AdcOffset;
    uint16_t *ADCValue;
    int Head;
    int Tail;
    uint8_t Buffer[RADIO_CIRCULAR_BUFFER_SIZE];
} RadioHandler_t;

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void Radio_Init(RadioHandler_t *Handle, RadioInit_t Init);
RadioStatus_t Radio_Process();
int Radio_Put_String(char *Buffer);
int Radio_Get_String(uint8_t *Buffer, int Size);
int Radio_available();
int Radio_Remain();
// Function for reading ADC
void Varires_Calib(VariRes_Value_t *vrOffset, uint16_t *AdcDMA,
                   uint16_t ui16Times);
// Function for reading ADC
void Calculate_PWM(VariRes_Value_t *ui16PWM, uint16_t *ui16Adc,
                   VariRes_Value_t *VrOffsetValue);
#endif /* __RADIO_H */
