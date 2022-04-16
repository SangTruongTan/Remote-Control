/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : Radio.c
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
#include "Radio.h"
/* Private includes ----------------------------------------------------------*/
#include <stdlib.h>
#include <string.h>

#include "RemoteControl.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define MIN(x, y) (((x) < (y)) ? (x) : (y))

#define RADIO_MAX_CMD_LENGTH 256
#define ROWS 32
#define COLUMNS 32

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
RadioHandler_t *RadioHandler;

uint64_t TxpipeAddrs = 0x11223344AA;

/* Private function prototypes -----------------------------------------------*/
void Radio_Process_From_FC(uint8_t *Buffer);
int Radio_Control_Parse(uint8_t (*TempBuffer)[COLUMNS], uint8_t *Source,
                        const char Deli);
void Radio_Decode_CMD(uint8_t (*Struct)[COLUMNS]);
void Radio_Decode_PID(uint8_t (*StructBuffer)[COLUMNS]);
void Radio_Decode_Ack(uint8_t (*StructBuffer)[COLUMNS]);
void Radio_Decode_Mode(uint8_t (*StructBuffer)[COLUMNS]);
// Function for Circular buffer.
void Radio_Increase_Pointer(int *Pointer, uint16_t NumbersBytes);
/* Function definations ------------------------------------------------------*/
/**
 * @brief Init the Radio task.
 * @param Handle The pointer of the handler.
 * @param Init The init of the radio.
 * @param void
 */
void Radio_Init(RadioHandler_t *Handle, RadioInit_t Init) {
    RadioHandler = Handle;
    RadioHandler->Init = Init;
    RadioHandler->enableDebug = Init.enableDebug;
    RadioHandler->Serial = Init.Serial;
    RadioHandler->Debug = Init.Debug;
    RadioHandler->Display = Init.Display;
    RadioHandler->ADCValue = Init.ADCValue;
    NRF24_begin(RadioHandler->Init.nrfInit);
    nrf24_DebugUART_Init(*Init.Debug->huart);
    NRF24_stopListening();
    NRF24_openWritingPipe(TxpipeAddrs);
    NRF24_setChannel(12);
    NRF24_setAutoAck(true);
    NRF24_setPayloadSize(32);
    NRF24_setPALevel(RF24_PA_m18dB);
    NRF24_setDataRate(RF24_2MBPS);
    NRF24_setRetries(0x07, 15);  // Delay 1250us and 15 times
    NRF24_enableAckPayload();
    NRF24_enableDynamicPayloads();
    printRadioSettings();
}

/**
 * @brief Process the Radio task. Called periodic.
 * @retval RadioStatus_t
 */
RadioStatus_t Radio_Process() {
    static uint8_t DataFromFC[RADIO_MAX_CMD_LENGTH];
    uint8_t *Buffer;
    Buffer = malloc(33);
    memset(Buffer, '\0', 33);
    // Process the Adc value
    Calculate_PWM(&RadioHandler->PWMValue, RadioHandler->ADCValue,
                  &RadioHandler->AdcOffset);
    // Check the data in the circular buffer.
    int Available = Radio_Get_String(Buffer, 32);
    if (Available > 0) {
        if (NRF24_write(Buffer, Available)) {
            // Successfully transmission
            if (NRF24_isAckPayloadAvailable()) {
                memset(Buffer, '\0', 33);
                NRF24_read(Buffer, 32);
                strcat((char *)DataFromFC, (char *)Buffer);
                // Process data from FC
                if (IndexOf(DataFromFC, '\n', strlen((char *)DataFromFC)) >=
                    0) {
                    Radio_Process_From_FC(DataFromFC);
                    memset(DataFromFC, '\0', RADIO_MAX_CMD_LENGTH);
                }
            }
        }
    }
    free(Buffer);
    return RADIO_OK;
}

/**
 * @brief Process the data from the FC.
 * @param Buffer The pointer of the buffer.
 * @retval void
 */
void Radio_Process_From_FC(uint8_t *Buffer) {
    if (RadioHandler->enableDebug == true) {
        HAL_UART_Transmit(RadioHandler->Debug->huart, Buffer,
                          strlen((char *)Buffer), 100);
    }
    if (IndexOf(Buffer, '\n', strlen((char *)Buffer)) > 0) {
        uint8_t Struct[ROWS][COLUMNS];
        // Put the process with the Parse string function
        Radio_Control_Parse(Struct, Buffer, ',');
        if (strcmp((const char *)&Struct[0], "MOD") == 0) {
            Radio_Decode_Mode(Struct);
        } else if (strcmp((const char *)&Struct[0], "ACK") == 0) {
            Radio_Decode_Ack(Struct);
        }
    }
}

/**
 * @brief Parse the string with the delimiter.
 * @param TempBuffer The temporary buffer.
 * @param Source The Source of the String.
 * @param Deli the delimiter.
 * @retval int The number of split string.
 */
int Radio_Control_Parse(uint8_t (*TempBuffer)[COLUMNS], uint8_t *Source,
                        const char Deli) {
    int length = 0;
    char *Head = malloc(32);
    Head = strtok((char *)Source, &Deli);
    while (Head != NULL) {
        strcpy((char *)&(TempBuffer)[length], Head);
        length++;
        Head = strtok(NULL, &Deli);
    }
    free(Head);
    return length;
}

/**
 * @brief Decode the CMD message.
 * @param Struct The Structural Buffer.
 * @retval ControlStatus_t
 */
void Radio_Decode_CMD(uint8_t (*Struct)[COLUMNS]) {
    int index = 1;
    Joystick_t *Control = &RadioHandler->Display->Joys;
    if (strcmp((const char *)&Struct[0], "CMD") != 0) return;
    while (Struct[index][0] != '\0') {
        if (strcmp((const char *)&Struct[index], "T") == 0) {
            Control->Thrust = atoi((const char *)&Struct[index + 1]);
        } else if (strcmp((const char *)&Struct[index], "R") == 0) {
            Control->Roll = atoi((const char *)&Struct[index + 1]);
        } else if (strcmp((const char *)&Struct[index], "P") == 0) {
            Control->Pitch = atoi((const char *)&Struct[index + 1]);
        } else if (strcmp((const char *)&Struct[index], "Y") == 0) {
            Control->Yaw = atoi((const char *)&Struct[index + 1]);
        }
        index += 2;
    }
}

/**
 * @brief Decode the PID message.
 * @param StructBuffer The Structural Buffer.
 * @retval void
 */
void Radio_Decode_PID(uint8_t (*StructBuffer)[COLUMNS]) {
    int index = 1;
    ControlPid_t *ControlPid = &RadioHandler->Display->Pid;
    if (strcmp((const char *)&StructBuffer[0], "PID") != 0) return;
    while (StructBuffer[index][0] != '\0') {
        if (strcmp((const char *)&StructBuffer[index], "PR") == 0) {
            ControlPid->Roll.P = atof((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "IR") == 0) {
            ControlPid->Roll.I = atof((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "DR") == 0) {
            ControlPid->Roll.D = atof((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "PP") == 0) {
            ControlPid->Pitch.P = atof((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "IP") == 0) {
            ControlPid->Pitch.I = atof((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "DP") == 0) {
            ControlPid->Pitch.D = atof((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "PY") == 0) {
            ControlPid->Yaw.P = atof((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "IY") == 0) {
            ControlPid->Yaw.I = atof((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "DY") == 0) {
            ControlPid->Yaw.D = atof((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "PA") == 0) {
            ControlPid->Altitude.P =
                atof((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "IA") == 0) {
            ControlPid->Altitude.I =
                atof((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "DA") == 0) {
            ControlPid->Altitude.D =
                atof((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "PG") == 0) {
            ControlPid->Gps.P = atof((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "IG") == 0) {
            ControlPid->Gps.I = atof((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "DG") == 0) {
            ControlPid->Gps.D = atof((const char *)&StructBuffer[index + 1]);
        }
        index += 2;
    }
}

/**
 * @brief Decode ACK message from Flight Controller.
 * @param Struct The pointer of the struct.
 * @retval void
 */
void Radio_Decode_Ack(uint8_t (*StructBuffer)[COLUMNS]) {
    int index = 1;
    SensorParameters_t *Parameter = &RadioHandler->Display->SystemParameters;
    if (strcmp((const char *)&StructBuffer[0], "ACK") != 0) return;
    while (StructBuffer[index][0] != '\0') {
        if (strcmp((const char *)&StructBuffer[index], "R") == 0) {
            Parameter->Roll = atof((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "P") == 0) {
            Parameter->Pitch = atof((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "Y") == 0) {
            Parameter->Yaw = atof((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "B") == 0) {
            Parameter->VBat = atof((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "A") == 0) {
            Parameter->Altitude = atof((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "L") == 0) {
            Parameter->Gps.Latitude =
                atof((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "O") == 0) {
            Parameter->Gps.Longitude =
                atof((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "S") == 0) {
            Parameter->Gps.Sattellites =
                atof((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "F") == 0) {
            if (StructBuffer[index + 1][0] == '1') {
                Parameter->Gps.Fixed = true;
            } else {
                Parameter->Gps.Fixed = false;
            }
        }
        index += 2;
    }
}

/**
 * @brief Decode Mode from the Flight Controller.
 * @param Struct The pointer of the struct.
 * @retval void
 */
void Radio_Decode_Mode(uint8_t (*StructBuffer)[COLUMNS]) {
    OLEDDisplay_t *dis = RadioHandler->Display;
    if (strcmp((const char *)&StructBuffer[0], "MOD") != 0) return;
    strcpy(dis->Mode, (const char *)&StructBuffer[1]);
}

// Function For Circular Buffer
/**
 * @brief Put the string into the circular buffer.
 * @param Buffer The pointer of the string.
 * @return int Number of bytes remaining in the buffer. Return -1 means the size
 * of the buffer doesn't enough.
 */
int Radio_Put_String(char *Buffer) {
    uint16_t StringLength = strlen(Buffer);
    uint16_t Head = RadioHandler->Head;
    uint16_t Tail = RadioHandler->Tail;
    // Check the size of the message
    if (StringLength > RADIO_CIRCULAR_BUFFER_SIZE) {
        return -1;
    }
    // Calculate remaining size
    int Remain = Tail - Head;
    if (Remain <= 0) {
        Remain += RADIO_CIRCULAR_BUFFER_SIZE;
    }
    // Check whether the messsage exceeds the remaining size.
    if (Head + StringLength > RADIO_CIRCULAR_BUFFER_SIZE) {
        uint16_t DataToCopy = RADIO_CIRCULAR_BUFFER_SIZE - Head;
        memcpy((RadioHandler->Buffer + Head), Buffer, DataToCopy);
        Head = 0;
        memcpy(RadioHandler->Buffer, (Buffer + DataToCopy),
               StringLength - DataToCopy);
        RadioHandler->Head = StringLength - DataToCopy;
    } else {
        memcpy((RadioHandler->Buffer + Head), Buffer, StringLength);
        RadioHandler->Head += StringLength;
    }
    if (StringLength >= Remain) {
        RadioHandler->Tail = RadioHandler->Head + 1;
    }
    // Calculate remaining again
    Remain = RadioHandler->Tail - RadioHandler->Head;
    if (Remain <= 0) {
        Remain += RADIO_CIRCULAR_BUFFER_SIZE;
    }
    return Remain;
}

/**
 * @brief Get the string from the buffer with particular size.
 * @param Buffer The pointer of the Buffer.
 * @param Size The size needs to read.
 * @return int The size of the string was read.
 */
int Radio_Get_String(uint8_t *Buffer, int Size) {
    int Available = Radio_available();
    if(Available > Size)
        Available = Size;
    int Readable = Available;
    if (Readable <= 0) return 0;
    int Tail = RadioHandler->Tail;
    int Remain = RADIO_CIRCULAR_BUFFER_SIZE - Tail;
    if (Readable > Remain) {
        memmove(Buffer, (const void *)&RadioHandler->Buffer[Tail], Remain);
        memmove(Buffer + Remain, (const void *)RadioHandler->Buffer,
               Readable - Remain);
    } else {
        memmove(Buffer, (const void *)&RadioHandler->Buffer[Tail], Readable);
    }
    memset(Buffer + Readable + 1, '\0', 1);
    Radio_Increase_Pointer(&RadioHandler->Tail, Readable);
    return Readable;
}

/**
 * @brief The remain space of the circular buffer.
 * @return int The number of bytes available.
 */
int Radio_Remain() {
    int Remain = RadioHandler->Tail - RadioHandler->Head;
    if (Remain <= 0) {
        Remain += RADIO_CIRCULAR_BUFFER_SIZE;
    }
    return Remain;
}

/**
 * @brief Get the available character in the buffer.
 * @return int Number of string are remaining.
 */
int Radio_available() {
    int Head = RadioHandler->Head;
    int Tail = RadioHandler->Tail;
    int available = 0;
    if (Head > Tail) {
        available = Head - Tail;
    } else if (Head < Tail) {
        available = RADIO_CIRCULAR_BUFFER_SIZE - Tail + Head;
    }
    return available;
}

/**
 * @brief Increase the pointer of the circular buffer.
 * @param Pointer The pointer.
 * @param NumbersBytes Numbers of bytes need to be increase.
 */
void Radio_Increase_Pointer(int *Pointer, uint16_t NumbersBytes) {
    if (*Pointer + NumbersBytes >= RADIO_CIRCULAR_BUFFER_SIZE) {
        *Pointer += NumbersBytes - RADIO_CIRCULAR_BUFFER_SIZE;
    } else {
        *Pointer += NumbersBytes;
    }
}

// Support function for VarRes
void Varires_Calib(VariRes_Value_t *vrOffset, uint16_t *AdcDMA,
                   uint16_t ui16Times) {
    uint16_t i;
    uint32_t buffer[4];
    memset(buffer, 0x00, sizeof(uint32_t) * 4);
    for (i = 0; i < ui16Times; i++) {
        buffer[0] += *AdcDMA;
        buffer[1] += *(AdcDMA + 1);
        buffer[2] += *(AdcDMA + 2);
        buffer[3] += *(AdcDMA + 3);
        HAL_Delay(20);
        /* USER CODE BEGIN */
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
        /* USER CODE End */
    }
    vrOffset->Adc1 = buffer[0] / ui16Times;
    vrOffset->Adc2 = buffer[1] / ui16Times;
    vrOffset->Adc3 = buffer[2] / ui16Times;
    vrOffset->Adc4 = buffer[3] / ui16Times;
}

void Calculate_PWM(VariRes_Value_t *ui16PWM, uint16_t *ui16Adc,
                   VariRes_Value_t *VrOffsetValue) {
    static int Temp[4];
    static int count = 1;
    if (count < 5) {
        for(int i = 0; i < 4; i++) {
            Temp[i] += ui16Adc[i];
        }
    } else {
        for(int i = 0; i < 4; i++) {
            Temp[i] /= 4;
        }
        // Calculate Adc value
        ui16PWM->Adc1 =
            (*Temp - VrOffsetValue->Adc1 + 2048) * 1000 / 4095 + 1000;
        ui16PWM->Adc2 =
            (*(Temp + 1) - VrOffsetValue->Adc2 + 2048) * 1000 / 4095 + 1000;
        ui16PWM->Adc4 =
            (*(Temp + 3) - VrOffsetValue->Adc4 + 2048) * 1000 / 4095 + 1000;
        ui16PWM->Adc3 =
            (*(Temp + 2) - VrOffsetValue->Adc3 + 0) * 1000 / 4095 + 1000;
        if (ui16PWM->Adc1 < 1000)
            ui16PWM->Adc1 = 1000;
        else if (ui16PWM->Adc1 > 2000)
            ui16PWM->Adc1 = 2000;
        if (ui16PWM->Adc2 < 1000)
            ui16PWM->Adc2 = 1000;
        else if (ui16PWM->Adc2 > 2000)
            ui16PWM->Adc2 = 2000;
        if (ui16PWM->Adc3 < 1000)
            ui16PWM->Adc3 = 1000;
        else if (ui16PWM->Adc3 > 2000)
            ui16PWM->Adc3 = 2000;
        if (ui16PWM->Adc4 < 1000)
            ui16PWM->Adc4 = 1000;
        else if (ui16PWM->Adc4 > 2000)
            ui16PWM->Adc4 = 2000;
        memset(Temp, 0, sizeof(uint32_t) * 4);
        count = 0;
    }
    count ++;
}
/* Private user code ---------------------------------------------------------*/
