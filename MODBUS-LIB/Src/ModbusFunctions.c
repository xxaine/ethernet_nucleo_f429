#include "Modbus.h"
#include "ModbusRegisters.h"
#include <stm32f429xx.h>
#include <stm32f4xx_hal_gpio.h>

// Функция 3: Чтение регистров (уже реализована)
// Функция 4: Чтение входных регистров
int8_t process_FC4(modbusHandler_t *modH) {
    uint16_t u16StartAdd = word(modH->u8Buffer[ADD_HI], modH->u8Buffer[ADD_LO]);
    uint16_t u16RegsNo = word(modH->u8Buffer[NB_HI], modH->u8Buffer[NB_LO]);

    /* Обработка Input Registers (1000-1019) */
    if (u16StartAdd >= 1000 && u16StartAdd + u16RegsNo <= 1019 + 1) {
        modH->u8Buffer[2] = u16RegsNo * 2;
        for (uint16_t i = 0; i < u16RegsNo; i++) {
            uint16_t value = Modbus_GetInputRegister(u16StartAdd + i);
            modH->u8Buffer[3 + i*2] = highByte(value);
            modH->u8Buffer[4 + i*2] = lowByte(value);
        }
        modH->u8BufferSize = 3 + u16RegsNo * 2;
        return modH->u8BufferSize + 2;
    }

    return EXC_ADDR_RANGE;
}

// Функция 16: Запись нескольких регистров (уже реализована) 