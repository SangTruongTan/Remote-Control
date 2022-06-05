/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>

#include "Oled.h"
#include "Radio.h"
#include "uartRingBufDMA.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "event_groups.h"
#include "queue.h"
#include "semphr.h"
#include "stdio.h"
#include "task.h"
#include "timers.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc2;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */

RingBuffer_t Ring;
RadioHandler_t Radio;
OLEDHandle_t Oled;

uint16_t ADCValue[4];

// Task Handle
TaskHandle_t RadioTask;
TaskHandle_t DisplayTask;
TaskHandle_t CalculateTask;
TaskHandle_t InitTask;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI3_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC2_Init(void);

/* USER CODE BEGIN PFP */
void Init_Task(void *pvParameters);
void Display_Task(void *pvParameters);
void Radio_Task(void *pvParameters);
void ReadData_Task(void *pvParameters);

// RTOS
void Radio_Task(void *pvParameters);
void Display_Task(void *pvParameters);
void Calculate_Task(void *pvParameters);
void Init_Task(void *pvParamters);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU
     * Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the
     * Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_SPI3_Init();
    MX_DMA_Init();
    MX_USART3_UART_Init();
    MX_ADC2_Init();
    /* USER CODE BEGIN 2 */
    HAL_GPIO_WritePin(Buzz_GPIO_Port, Buzz_Pin, 1);
    HAL_Delay(200);
    HAL_GPIO_WritePin(Buzz_GPIO_Port, Buzz_Pin, 0);
    // For RTOS
    xTaskCreate(Init_Task, "Init", 256, NULL, 3, &InitTask);
    vTaskStartScheduler();
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 4;
    RCC_OscInitStruct.PLL.PLLN = 192;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV6;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * @brief ADC2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC2_Init(void) {
    /* USER CODE BEGIN ADC2_Init 0 */

    /* USER CODE END ADC2_Init 0 */

    ADC_ChannelConfTypeDef sConfig = {0};

    /* USER CODE BEGIN ADC2_Init 1 */

    /* USER CODE END ADC2_Init 1 */
    /** Configure the global features of the ADC (Clock, Resolution, Data
     * Alignment and number of conversion)
     */
    hadc2.Instance = ADC2;
    hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
    hadc2.Init.Resolution = ADC_RESOLUTION_12B;
    hadc2.Init.ScanConvMode = ENABLE;
    hadc2.Init.ContinuousConvMode = ENABLE;
    hadc2.Init.DiscontinuousConvMode = DISABLE;
    hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc2.Init.NbrOfConversion = 4;
    hadc2.Init.DMAContinuousRequests = ENABLE;
    hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&hadc2) != HAL_OK) {
        Error_Handler();
    }
    /** Configure for the selected ADC regular channel its corresponding rank in
     * the sequencer and its sample time.
     */
    sConfig.Channel = ADC_CHANNEL_4;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
        Error_Handler();
    }
    /** Configure for the selected ADC regular channel its corresponding rank in
     * the sequencer and its sample time.
     */
    sConfig.Channel = ADC_CHANNEL_5;
    sConfig.Rank = 2;
    if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
        Error_Handler();
    }
    /** Configure for the selected ADC regular channel its corresponding rank in
     * the sequencer and its sample time.
     */
    sConfig.Channel = ADC_CHANNEL_6;
    sConfig.Rank = 3;
    if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
        Error_Handler();
    }
    /** Configure for the selected ADC regular channel its corresponding rank in
     * the sequencer and its sample time.
     */
    sConfig.Channel = ADC_CHANNEL_7;
    sConfig.Rank = 4;
    if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN ADC2_Init 2 */

    /* USER CODE END ADC2_Init 2 */
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {
    /* USER CODE BEGIN I2C1_Init 0 */

    /* USER CODE END I2C1_Init 0 */

    /* USER CODE BEGIN I2C1_Init 1 */

    /* USER CODE END I2C1_Init 1 */
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 400000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN I2C1_Init 2 */

    /* USER CODE END I2C1_Init 2 */
}

/**
 * @brief SPI3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI3_Init(void) {
    /* USER CODE BEGIN SPI3_Init 0 */

    /* USER CODE END SPI3_Init 0 */

    /* USER CODE BEGIN SPI3_Init 1 */

    /* USER CODE END SPI3_Init 1 */
    /* SPI3 parameter configuration*/
    hspi3.Instance = SPI3;
    hspi3.Init.Mode = SPI_MODE_MASTER;
    hspi3.Init.Direction = SPI_DIRECTION_2LINES;
    hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi3.Init.NSS = SPI_NSS_SOFT;
    hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi3.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi3) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN SPI3_Init 2 */

    /* USER CODE END SPI3_Init 2 */
}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void) {
    /* USER CODE BEGIN USART3_Init 0 */

    /* USER CODE END USART3_Init 0 */

    /* USER CODE BEGIN USART3_Init 1 */

    /* USER CODE END USART3_Init 1 */
    huart3.Instance = USART3;
    huart3.Init.BaudRate = 115200;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN USART3_Init 2 */

    /* USER CODE END USART3_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {
    /* DMA controller clock enable */
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    /* DMA interrupt init */
    /* DMA1_Stream1_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
    /* DMA1_Stream3_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
    /* DMA2_Stream3_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, LED_Pin | Buzz_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOC, RF_CE_Pin | RF_CS_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(BTN_R_Ver_GPIO_Port, BTN_R_Ver_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pins : AUX_TRIM_Pin SW_RIGHT_Pin GEAR_Pin BTN1_LCD_Pin
                             BTN2_LCD_Pin BTN3_LCD_Pin BTN4_LCD_Pin BTN_ENC_Pin
     */
    GPIO_InitStruct.Pin = AUX_TRIM_Pin | SW_RIGHT_Pin | GEAR_Pin |
                          BTN1_LCD_Pin | BTN2_LCD_Pin | BTN3_LCD_Pin |
                          BTN4_LCD_Pin | BTN_ENC_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pins : Right_Ver_Pin Right_Hor_Pin Left_Ver_Pin
     * Left_Hor_Pin */
    GPIO_InitStruct.Pin =
        Right_Ver_Pin | Right_Hor_Pin | Left_Ver_Pin | Left_Hor_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pins : BTN_L_Ver_Pin FLAP_TRIM_Pin FLAP_Pin */
    GPIO_InitStruct.Pin = BTN_L_Ver_Pin | FLAP_TRIM_Pin | FLAP_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pins : LED_Pin Buzz_Pin */
    GPIO_InitStruct.Pin = LED_Pin | Buzz_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pins : RF_CE_Pin RF_CS_Pin */
    GPIO_InitStruct.Pin = RF_CE_Pin | RF_CS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pins : BTN_R_Hor_R_Pin BTN_R_Hor_L_Pin BTN_L_Hor_R_Pin
     * BTN_L_Hor_L_Pin */
    GPIO_InitStruct.Pin =
        BTN_R_Hor_R_Pin | BTN_R_Hor_L_Pin | BTN_L_Hor_R_Pin | BTN_L_Hor_L_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin : BTN_R_Ver_Pin */
    GPIO_InitStruct.Pin = BTN_R_Ver_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BTN_R_Ver_GPIO_Port, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
int _write(int file, char *outgoing, int len) {
    HAL_UART_Transmit(&huart3, (uint8_t *)outgoing, len, 100);
    return len;
}

void Init_Task(void *pvParameters) {
    // Init for ADC with DMA
    HAL_ADC_Start_DMA(&hadc2, (uint32_t *)ADCValue, 4);
    // Init for the Ring Buffer. Must be call before other application
    Ring.Ring1.enable = true;
    Ring.Ring1.hdma = &hdma_usart3_rx;
    Ring.Ring1.huart = &huart3;
    Ring.GetTime = xTaskGetTickCount;
    Ring_Init(&Ring);
    // Init for the Radio SubSystem
    Radio.Init.Debug = &Ring.Ring1;
    Radio.Init.enableDebug = false;
    Radio.Init.nrfInit.CE_Pin = RF_CE_Pin;
    Radio.Init.nrfInit.CSN_Pin = RF_CS_Pin;
    Radio.Init.nrfInit.Port = RF_CS_GPIO_Port;
    Radio.Init.nrfInit.getTick = xTaskGetTickCount;
    Radio.Init.nrfInit.Timeout = 0;
    Radio.Init.nrfInit.wait = vTaskDelay;
    Radio.Init.nrfInit.hspi = &hspi3;
    Radio.Init.nrfInit.LostPackages = &Radio.LostPackages;
    Radio.Init.Serial = &Ring.Ring1;
    Radio.Init.Display = &Oled.Display;
    Radio.Init.ADCValue = ADCValue;
    Radio_Init(&Radio, Radio.Init);

    // Put Adc offset or Calibration
    Radio.AdcOffset.Adc1 = 2059;
    Radio.AdcOffset.Adc2 = 2181;
    Radio.AdcOffset.Adc3 = 289;
    Radio.AdcOffset.Adc4 = 1961;
    // Varires_Calib(&Radio.AdcOffset, ADCValue, 100);
    // Init for OLED Display
    Oled.Init.ControlPin.Pin = BTN1_LCD_Pin;
    Oled.Init.ControlPin.Port = BTN1_LCD_GPIO_Port;
    Oled.Init.ControlPin.AutoChange = false;
    Oled.Init.SelectPage = 0;
    Oled.Init.GetTime = xTaskGetTickCount;
    Oled.Init.Wait = vTaskDelay;
    Oled.Init.Heading = &Radio.Heading;
    Oled.Init.LostPackages = &Radio.LostPackages;
    Oled_Init(&Oled);
    xTaskCreate(Display_Task, "Display", 512, NULL, 2, &DisplayTask);
    xTaskCreate(Radio_Task, "Radio", 1024, NULL, 2, &RadioTask);
    xTaskCreate(Calculate_Task, "Calculate", 256, NULL, 2, &CalculateTask);
    vTaskDelete(InitTask);
    for (;;) {
    }
}

void Display_Task(void *pvParameters) {
    TickType_t StartTask = xTaskGetTickCount();
    vTaskDelay(1000);
    for (;;) {
        Oled_Process();
        // printf("Lost:%d\r\n", Radio.LostPackages);
        vTaskDelayUntil(&StartTask, 200);
    }
}

void Radio_Task(void *pvParameters) {
    char Buffer[64];
    TickType_t StartTask = xTaskGetTickCount();
    char Data[50];
    uint8_t count = 0;
    for (;;) {
        // Read the button status
        if (HAL_GPIO_ReadPin(SW_RIGHT_GPIO_Port, SW_RIGHT_Pin) ==
            GPIO_PIN_SET) {
            Radio.Heading = 'T';
        } else {
            Radio.Heading = 'F';
        }
        memset(Buffer, '\0', 64);
        if (Get_String_NonBlocking(&Ring.Ring1, (uint8_t *)Buffer, '\n') > 0) {
            strcpy(Data, Buffer);
            Data[strlen((char *)Data)] = '\n';
        } else {
            sprintf(Data, "CMD,T,%d,R,%d,P,%d,Y,%d,H,%c\n", Radio.PWMValue.Adc3,
                    Radio.PWMValue.Adc2, Radio.PWMValue.Adc1,
                    Radio.PWMValue.Adc4, Radio.Heading);
        }
        if (count == 0) {
            if (Radio_Remain() > strlen(Data)) Radio_Put_String(Data);
        }
        if (count != 0) {
            count++;
        } else if (count > 5) {
            count = 0;
        }
        Radio_Process();
        vTaskDelayUntil(&StartTask, 10);
    }
}

void Calculate_Task(void *pvParameters) {
    TickType_t StartTask = xTaskGetTickCount();
    for (;;) {
        Calculate_PWM(&Radio.PWMValue, Radio.ADCValue, &Radio.AdcOffset);
        vTaskDelayUntil(&StartTask, 2);
    }
}

/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM5 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    /* USER CODE BEGIN Callback 0 */

    /* USER CODE END Callback 0 */
    if (htim->Instance == TIM5) {
        HAL_IncTick();
    }
    /* USER CODE BEGIN Callback 1 */

    /* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state
     */
    __disable_irq();
    while (1) {
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
void assert_failed(uint8_t *file, uint32_t line) {
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line
       number, ex: printf("Wrong parameters value: file %s on line %d\r\n",
       file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
