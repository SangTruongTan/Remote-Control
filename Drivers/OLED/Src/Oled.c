/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : File.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "Oled.h"

/* Private includes ----------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ssd1306.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define MAX_PAGE_DISPLAY 2
#define PIN_ACTIVE 1
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
OLEDHandle_t *OledHandler;

/* Private function prototypes -----------------------------------------------*/
/**
 * @brief Initialize the Oled subsystem.
 * @param Handle The pointer of the OLED handler.
 * @retval OLEDStatus_t
 */
OLEDStatus_t Oled_Init(OLEDHandle_t *Handle) {
    Handle->ControlPin = Handle->Init.ControlPin;
    Handle->SelectPage = Handle->Init.SelectPage;
    OledHandler = Handle;
    ssd1306_Init();
    // Put the information to introduce
    const char *Info = "DRONE";
    const char *Info1 = "4G";
    ssd1306_SetCursor(25, 0);
    ssd1306_WriteString((char *)Info, Font_16x26, White);
    ssd1306_SetCursor(50, 30);
    ssd1306_WriteString((char *)Info1, Font_16x26, White);
    ssd1306_UpdateScreen();
    return OLED_OK;
}

/**
 * @brief Process the OLED display data. Recommend with period 200ms.
 * @retval OLEDStatus_t
 */
OLEDStatus_t Oled_Process(void) {
    char *Temp = NULL;
    static uint8_t LiveDisplay = 0;
    Temp = malloc(64);
    // Change the page selection to display
    if (OledHandler->ControlPin.AutoChange == true) {
    } else {
        if (HAL_GPIO_ReadPin(OledHandler->ControlPin.Port,
                             OledHandler->ControlPin.Pin) == PIN_ACTIVE) {
            OledHandler->SelectPage += 1;
            if (OledHandler->SelectPage > MAX_PAGE_DISPLAY)
                OledHandler->SelectPage = 0;
        }
    }
    // Display the data rely selection display
    if (OledHandler->SelectPage == 0) {
        sprintf(Temp, "Display Page:%d", OledHandler->SelectPage);
        ssd1306_Fill(Black);
        ssd1306_SetCursor(2, 0);
        ssd1306_WriteString(Temp, Font_7x10, White);
        ssd1306_UpdateScreen();
    } else if (OledHandler->SelectPage == 1) {
        OLEDDisplay_t dis = OledHandler->Display;
        ssd1306_Fill(Black);
        // Title
        sprintf(Temp, "Remote Control");
        ssd1306_SetCursor(15, 0);
        ssd1306_WriteString(Temp, Font_7x10, White);
        // For data
        sprintf(Temp, "R:%04.1f P:%04.1f", dis.SystemParameters.Roll,
                dis.SystemParameters.Pitch);
        ssd1306_SetCursor(2, 20);
        ssd1306_WriteString(Temp, Font_7x10, White);
        sprintf(Temp, "Y:%04.1f M:%s", dis.SystemParameters.Yaw, dis.Mode);
        ssd1306_SetCursor(2, 32);
        ssd1306_WriteString(Temp, Font_7x10, White);
        sprintf(Temp, "B:%02.1fV", dis.SystemParameters.VBat);
        ssd1306_SetCursor(2, 44);
        ssd1306_WriteString(Temp, Font_7x10, White);
        if (OledHandler->Display.RadioState == ALIVE) {
            if (LiveDisplay % 2 == 0) {
                sprintf(Temp, "S:LIVE");
            } else {
                sprintf(Temp, "S:    ");
            }
            LiveDisplay++;
        } else {
            if (LiveDisplay % 2 == 0) {
                sprintf(Temp, "S:DEAD");
            } else {
                sprintf(Temp, "S:    ");
            }
            LiveDisplay++;
        }
        ssd1306_SetCursor(58, 44);
        ssd1306_WriteString(Temp, Font_7x10, White);

        // Don't forget update the screen
        ssd1306_UpdateScreen();
    } else if (OledHandler->SelectPage == 2) {
        OLEDDisplay_t dis = OledHandler->Display;
        ssd1306_Fill(Black);
        // Title
        sprintf(Temp, "Flight Control");
        ssd1306_SetCursor(15, 0);
        ssd1306_WriteString(Temp, Font_7x10, White);
        // For Data
        sprintf(Temp, "R:%04.1f P:%04.1f", dis.SystemParameters.Roll,
                dis.SystemParameters.Pitch);
        ssd1306_SetCursor(2, 20);
        ssd1306_WriteString(Temp, Font_7x10, White);
        sprintf(Temp, "Y:%04.1f A:%04.1fm", dis.SystemParameters.Yaw,
                dis.SystemParameters.Altitude);
        ssd1306_SetCursor(2, 32);
        ssd1306_WriteString(Temp, Font_7x10, White);
        // Don't forget update the screen
        ssd1306_UpdateScreen();
    }
    // Put the Page Processing here
    free(Temp);
    return OLED_OK;
}

/* Private user code ---------------------------------------------------------*/
