/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f2xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define AUX_TRIM_Pin GPIO_PIN_0
#define AUX_TRIM_GPIO_Port GPIOC
#define SW_Right_Pin GPIO_PIN_1
#define SW_Right_GPIO_Port GPIOC
#define GEAR_Pin GPIO_PIN_2
#define GEAR_GPIO_Port GPIOC
#define BTN1_LCD_Pin GPIO_PIN_3
#define BTN1_LCD_GPIO_Port GPIOC
#define Left_VAR_Pin GPIO_PIN_1
#define Left_VAR_GPIO_Port GPIOA
#define Right_VAR_Pin GPIO_PIN_2
#define Right_VAR_GPIO_Port GPIOA
#define ENC_LCD_Pin GPIO_PIN_3
#define ENC_LCD_GPIO_Port GPIOA
#define Right_Ver_Pin GPIO_PIN_4
#define Right_Ver_GPIO_Port GPIOA
#define Right_Hor_Pin GPIO_PIN_5
#define Right_Hor_GPIO_Port GPIOA
#define Left_Ver_Pin GPIO_PIN_6
#define Left_Ver_GPIO_Port GPIOA
#define Left_Hor_Pin GPIO_PIN_7
#define Left_Hor_GPIO_Port GPIOA
#define BTN2_LCD_Pin GPIO_PIN_4
#define BTN2_LCD_GPIO_Port GPIOC
#define BTN3_LCD_Pin GPIO_PIN_5
#define BTN3_LCD_GPIO_Port GPIOC
#define BTN_L_Ver_Pin GPIO_PIN_0
#define BTN_L_Ver_GPIO_Port GPIOB
#define MCU_TX_Pin GPIO_PIN_10
#define MCU_TX_GPIO_Port GPIOB
#define MCU_RX_Pin GPIO_PIN_11
#define MCU_RX_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOB
#define BTN4_LCD_Pin GPIO_PIN_6
#define BTN4_LCD_GPIO_Port GPIOC
#define ENC_BTN_Pin GPIO_PIN_7
#define ENC_BTN_GPIO_Port GPIOC
#define RF_CE_Pin GPIO_PIN_8
#define RF_CE_GPIO_Port GPIOC
#define RF_CS_Pin GPIO_PIN_9
#define RF_CS_GPIO_Port GPIOC
#define BTN_R_Hor_R_Pin GPIO_PIN_8
#define BTN_R_Hor_R_GPIO_Port GPIOA
#define BTN_R_Hor_L_Pin GPIO_PIN_9
#define BTN_R_Hor_L_GPIO_Port GPIOA
#define BTN_L_Hor_R_Pin GPIO_PIN_10
#define BTN_L_Hor_R_GPIO_Port GPIOA
#define BTN_L_Hor_L_Pin GPIO_PIN_11
#define BTN_L_Hor_L_GPIO_Port GPIOA
#define BTN_R_Ver_Pin GPIO_PIN_15
#define BTN_R_Ver_GPIO_Port GPIOA
#define RF_SCK_Pin GPIO_PIN_10
#define RF_SCK_GPIO_Port GPIOC
#define RF_MISO_Pin GPIO_PIN_11
#define RF_MISO_GPIO_Port GPIOC
#define RF_MOSI_Pin GPIO_PIN_12
#define RF_MOSI_GPIO_Port GPIOC
#define Buzz_Pin GPIO_PIN_5
#define Buzz_GPIO_Port GPIOB
#define OLED_SCL_Pin GPIO_PIN_6
#define OLED_SCL_GPIO_Port GPIOB
#define OLED_SDA_Pin GPIO_PIN_7
#define OLED_SDA_GPIO_Port GPIOB
#define FLAP_TRIM_Pin GPIO_PIN_8
#define FLAP_TRIM_GPIO_Port GPIOB
#define FLAP_Pin GPIO_PIN_9
#define FLAP_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
