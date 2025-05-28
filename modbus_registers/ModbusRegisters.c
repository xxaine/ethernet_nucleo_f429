#include "ModbusRegisters.h"
#include <string.h>
#include "main.h"
#include "Modbus.h"

// Определение переменных
modbusHandler_t ModbusH;

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
    
    // Установка значений по умолчанию для микроконтроллера
    //holdingRegisters[SP_Delay_Before - 2000] = 0;    // 0 мс
    //holdingRegisters[SP_Pulse_Lenght - 2000] = 2000; // 20.00 мс
    //holdingRegisters[SP_Front_Type - 2000] = 0;      // Передний фронт
    holdingRegisters[SP_Power_27_V - 2000] = 0;      // 27V выключен
    holdingRegisters[SP_Pos_Count_Max - 2000] = encoderPulsesPerRevolution; // 2500 имп/об
    
    // Удалена логика управления PB1 при инициализации
    
    // Удалена логика обновления регистра обратной связи для PB1
}

uint16_t Modbus_GetInputRegister(InputRegisters reg) {
    // Обработка регистров микроконтроллера
    if (reg >= 1000 && reg <= 1019) {
        // Специальная обработка для FBK_Power_27_V - читаем текущее состояние пина
        if (reg == FBK_Power_27_V) {
            return (HAL_GPIO_ReadPin(control_27V_GPIO_Port, control_27V_Pin) == GPIO_PIN_SET) ? 1 : 0;
        }
        return inputRegisters[reg - 1000];
    }
    return 0xFFFF; // Ошибка: неверный адрес
}

void Modbus_SetHoldingRegister(HoldingRegisters reg, uint16_t value) {
    // Обработка регистров микроконтроллера
    if (reg >= 2000 && reg <= 2019) {
        holdingRegisters[reg - 2000] = value;
        
        // Специальная обработка для управления 27V
        if (reg == SP_Power_27_V) {
            // Управление выходом PB1 (логика инвертирована: 0 = включено, 1 = выключено)
            if (value == 0) {
                HAL_GPIO_WritePin(control_27V_GPIO_Port, control_27V_Pin, GPIO_PIN_RESET); // Включаем (устанавливаем 0)
            } else {
                HAL_GPIO_WritePin(control_27V_GPIO_Port, control_27V_Pin, GPIO_PIN_SET);   // Выключаем (устанавливаем 1)
            }
            inputRegisters[FBK_Power_27_V - 1000] = value; // Обновляем статус
        }
        // Обработка настройки количества импульсов на оборот
        if (reg == SP_Pos_Count_Max) {
             if (value > 0) { // Проверяем, что значение положительное
                holdingRegisters[reg - 2000] = value; // Обновляем holding регистр
                encoderPulsesPerRevolution = value;
                inputRegisters[FBK_Pos_Count_Max - 1000] = value;
            }
        }
        else
        {
             holdingRegisters[reg - 2000] = value; // Для остальных регистров просто обновляем значение
        }
    }
}