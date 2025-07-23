#include "ModbusRegisters.h"
#include <string.h>
#include "main.h"
#include "Modbus.h"

// Определение регистров
uint16_t inputRegisters[INPUT_REGISTERS_COUNT] = {0};
uint16_t holdingRegisters[HOLDING_REGISTERS_COUNT] = {0};

// Глобальные переменные для энкодера
int32_t encoderPosition = 0;
int32_t encoderPulsesPerRevolution = 2500; // По умолчанию 2500 имп/об
int32_t encoderSpeed = 0;

void ModbusRegisters_Init(void) {
    memset(inputRegisters, 0, sizeof(inputRegisters));
    memset(holdingRegisters, 0, sizeof(holdingRegisters));
    
    // Установка значений по умолчанию
    holdingRegisters[SP_Delay_Before - 2000] = 0;     // Задержка 0 мс
    holdingRegisters[SP_Pulse_Lenght - 2000] = 2000; // Длительность импульса 20 мс
    holdingRegisters[SP_Front_Type - 2000] = 0;       // Передний фронт
    holdingRegisters[SP_Power_27_V - 2000] = 1;       // 27V выключен
    holdingRegisters[SP_Pos_Count_Max - 2000] = encoderPulsesPerRevolution; // 2500 имп/об
    holdingRegisters[SP_Pulse_On - 2000] = 1; // Разрешить генерацию импульсов по умолчанию
    
    // Инициализация input регистров
    inputRegisters[FBK_Delay_Before - 1000] = holdingRegisters[SP_Delay_Before - 2000];
    inputRegisters[FBK_Pulse_Lenght - 1000] = holdingRegisters[SP_Pulse_Lenght - 2000];
    inputRegisters[FBK_Front_Type - 1000] = holdingRegisters[SP_Front_Type - 2000];
    inputRegisters[FBK_Power_27_V - 1000] = holdingRegisters[SP_Power_27_V - 2000];
    inputRegisters[FBK_Pulse_On - 1000] = holdingRegisters[SP_Pulse_On - 2000];
    inputRegisters[FBK_Pos_Count_Max - 1000] = holdingRegisters[SP_Pos_Count_Max - 2000];
    inputRegisters[FBK_Pos_Count - 1000] = 0;
    inputRegisters[FBK_Pulse_Count - 1000] = 0;
    inputRegisters[FBK_Pulse_On - 1000] = 0;
    inputRegisters[FBK_POS_Set - 1000] = 0;
}

uint16_t Modbus_GetInputRegister(InputRegisters reg) {
    if (reg >= 1000 && reg <= 1019) {
        switch(reg) {
            case FBK_Power_27_V:
                return (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_RESET) ? 1 : 0;
            default:
                // Для всех остальных регистров просто возвращаем значение из массива
                return inputRegisters[reg - 1000];
        }
    }
    return 0xFFFF; // Ошибка: неверный адрес
}

void Modbus_SetHoldingRegister(HoldingRegisters reg, uint16_t value) {
    if (reg >= 2000 && reg <= 2019) {
        switch(reg) {
            case SP_Power_27_V:
                holdingRegisters[reg - 2000] = value;
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, value ? GPIO_PIN_SET : GPIO_PIN_RESET);
                inputRegisters[FBK_Power_27_V - 1000] = value;
                break;
                
            case SP_Pos_Count_Max:
                if (value > 0) {
                    holdingRegisters[reg - 2000] = value;
                    encoderPulsesPerRevolution = value;
                    inputRegisters[FBK_Pos_Count_Max - 1000] = value;
                }
                break;
                
            case SP_Pulse_On:
                holdingRegisters[reg - 2000] = value;
                inputRegisters[reg - 1000] = value;
                if (value == 0) {
                    HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
                    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_15); // Очистить pending bit
                    inputRegisters[FBK_Pulse_On - 1000] = 0;
                    inputRegisters[FBK_Pulse_Count - 1000] = 0;
                } else {
                    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
                    inputRegisters[FBK_Pulse_On - 1000] = 1;
                }
                break;
            case SP_Delay_Before:
            case SP_Pulse_Lenght:
                holdingRegisters[reg - 2000] = value;
                inputRegisters[reg - 1000] = value;
                // Не трогаем EXTI15_10_IRQn
                break;
            case SP_Front_Type:
                holdingRegisters[reg - 2000] = value;
                inputRegisters[reg - 1000] = value;
                Sensor_Init(); // Повторная инициализация пина только при изменении фронта
                break;
            case SP_POS_Set:
                // Реализация: по фронту 0->1 записать encoderPosition в SP_Pos_Count
                static uint16_t prev_pos_set = 0;
                if (prev_pos_set == 0 && value == 1) {
                    holdingRegisters[SP_Pos_Count - 2000] = (uint16_t)(encoderPosition & 0xFFFF);
                }
                prev_pos_set = value;
                holdingRegisters[reg - 2000] = value;
                inputRegisters[reg - 1000] = value;
                break;
                
            default:
                holdingRegisters[reg - 2000] = value; // Для резервных регистров
                break;
        }
    }
}