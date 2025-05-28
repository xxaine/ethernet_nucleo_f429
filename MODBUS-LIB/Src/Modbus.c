#include "Modbus.h"
#include "ModbusRegisters.h"
#include <stm32f429xx.h>
#include <stm32f4xx_hal_gpio.h>
#include "main.h"

// ... (остальные include и определения)

int8_t process_FC3(modbusHandler_t *modH) {
    uint16_t u16StartAdd = word(modH->u8Buffer[ADD_HI], modH->u8Buffer[ADD_LO]);
    uint16_t u16CoilsNo = word(modH->u8Buffer[NB_HI], modH->u8Buffer[NB_LO]);

    /* Обработка Input Registers (1000-1019) */
    if (u16StartAdd >= 1000 && u16StartAdd + u16CoilsNo <= 1019 + 1) {
        modH->u8Buffer[2] = u16CoilsNo * 2;
        for (uint16_t i = 0; i < u16CoilsNo; i++) {
            uint16_t value = Modbus_GetInputRegister(u16StartAdd + i);
            modH->u8Buffer[3 + i*2] = highByte(value);
            modH->u8Buffer[4 + i*2] = lowByte(value);
        }
        modH->u8BufferSize = 3 + u16CoilsNo * 2;
        return modH->u8BufferSize + 2;
    }

    return EXC_ADDR_RANGE;
}

int8_t process_FC16(modbusHandler_t *modH) {
    uint16_t u16StartAdd = word(modH->u8Buffer[ADD_HI], modH->u8Buffer[ADD_LO]);
    uint16_t u16CoilsNo = word(modH->u8Buffer[NB_HI], modH->u8Buffer[NB_LO]);

    /* Обработка Holding Registers (2000-2019) */
    if (u16StartAdd >= 2000 && u16StartAdd + u16CoilsNo <= 2019 + 1) {
        for (uint16_t i = 0; i < u16CoilsNo; i++) {
            uint16_t value = word(
                modH->u8Buffer[BYTE_CNT + 1 + i*2],
                modH->u8Buffer[BYTE_CNT + 2 + i*2]
            );
            
            /* Специальная обработка для управления 27V */
            if (u16StartAdd + i == SP_Power_27_V) {
                // Управление выходом PB1 (логика инвертирована: 0 = включено, 1 = выключено)
                if (value == 0) {
                    HAL_GPIO_WritePin(control_27V_GPIO_Port, control_27V_Pin, GPIO_PIN_RESET); // Включаем (устанавливаем 0)
                } else {
                    HAL_GPIO_WritePin(control_27V_GPIO_Port, control_27V_Pin, GPIO_PIN_SET);   // Выключаем (устанавливаем 1)
                }
                Modbus_SetHoldingRegister(u16StartAdd + i, value); // Обновляем регистр
            } else {
                Modbus_SetHoldingRegister(u16StartAdd + i, value);
            }
        }
        modH->u8BufferSize = 6;
        sendTxBuffer(modH);
        return modH->u8BufferSize + 2;
    }
    return EXC_ADDR_RANGE;
}

// Реализация функции отправки буфера
void sendTxBuffer(modbusHandler_t *modH) {
    if (modH->uModbusType == MB_SLAVE) {
        // Для slave просто отправляем ответ
        HAL_UART_Transmit(modH->port, modH->u8Buffer, modH->u8BufferSize, 100);
    }
}

// Вспомогательные функции
uint8_t highByte(uint16_t w) {
    return (uint8_t)((w >> 8) & 0xFF);
}

uint8_t lowByte(uint16_t w) {
    return (uint8_t)(w & 0xFF);
}

uint16_t word(uint8_t h, uint8_t l) {
    return (uint16_t)((h << 8) | l);
}

// ... (остальные функции модуля остаются без изменений)