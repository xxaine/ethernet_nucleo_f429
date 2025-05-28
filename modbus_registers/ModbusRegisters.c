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
    holdingRegisters[SP_Pulse_Lenght - 2000] = 20000; // Длительность импульса 20 мс
    holdingRegisters[SP_Front_Type - 2000] = 0;       // Передний фронт
    holdingRegisters[SP_Power_27_V - 2000] = 0;       // 27V выключен
    holdingRegisters[SP_Pos_Count_Max - 2000] = encoderPulsesPerRevolution; // 2500 имп/об
    
    // Инициализация input регистров
    inputRegisters[FBK_Delay_Before - 1000] = holdingRegisters[SP_Delay_Before - 2000];
    inputRegisters[FBK_Pulse_Lenght - 1000] = holdingRegisters[SP_Pulse_Lenght - 2000];
    inputRegisters[FBK_Front_Type - 1000] = holdingRegisters[SP_Front_Type - 2000];
    inputRegisters[FBK_Power_27_V - 1000] = holdingRegisters[SP_Power_27_V - 2000];
    inputRegisters[FBK_Pos_Count_Max - 1000] = holdingRegisters[SP_Pos_Count_Max - 2000];
}

uint16_t Modbus_GetInputRegister(InputRegisters reg) {
    if (reg >= 1000 && reg <= 1019) {
        switch(reg) {
            case FBK_Power_27_V:
                return (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_RESET) ? 1 : 0;
            case FBK_Delay_Before:
            case FBK_Pulse_Lenght:
            case FBK_Front_Type:
            case FBK_Pos_Count:
            case FBK_Pos_Count_Max:
            case FBK_Pulse_Count:
            case FBK_Pulse_On:
            case FBK_POS_Set:
        return inputRegisters[reg - 1000];
            default:
                return 0; // Для резервных регистров
        }
    }
    return 0xFFFF; // Ошибка: неверный адрес
}

void Modbus_SetHoldingRegister(HoldingRegisters reg, uint16_t value) {
    if (reg >= 2000 && reg <= 2019) {
        switch(reg) {
            case SP_Power_27_V:
                holdingRegisters[reg - 2000] = value;
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, value ? GPIO_PIN_RESET : GPIO_PIN_SET);
                inputRegisters[FBK_Power_27_V - 1000] = value;
                break;
                
            case SP_Pos_Count_Max:
                if (value > 0) {
                    holdingRegisters[reg - 2000] = value;
                encoderPulsesPerRevolution = value;
                inputRegisters[FBK_Pos_Count_Max - 1000] = value;
            }
                break;
                
            case SP_Delay_Before:
            case SP_Pulse_Lenght:
            case SP_Front_Type:
            case SP_Pulse_On:
            case SP_POS_Set:
                holdingRegisters[reg - 2000] = value;
                inputRegisters[reg - 1000] = value; // Обновляем соответствующий input регистр
                break;
                
            default:
                holdingRegisters[reg - 2000] = value; // Для резервных регистров
                break;
        }
    }
}