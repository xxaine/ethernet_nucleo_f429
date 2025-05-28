#include "Modbus.h"
#include "ModbusRegisters.h"

// Глобальные переменные для обработчиков Modbus
modbusHandler_t *mHandlers[MAX_M_HANDLERS];
uint8_t numberHandlers = 0;

// Объявления функций
int8_t process_FC3(modbusHandler_t *modH);
int8_t process_FC16(modbusHandler_t *modH);
void sendTxBuffer(modbusHandler_t *modH);

void Modbus_InitSlave(modbusHandler_t *modH) {
    if (numberHandlers < MAX_M_HANDLERS) {
        modH->uModbusType = MB_SLAVE;
        modH->u8id = 1;  // Slave ID
        modH->u16regsize = 0;
        modH->u16InCnt = 0;
        modH->u16OutCnt = 0;
        modH->u16errCnt = 0;
        modH->u8BufferSize = 0;
        modH->u8lastRec = 0;
        modH->i8state = 0;
        modH->i8lastError = 0;
        modH->u8AddressMode = ADDRESS_NORMAL;
        
        // Инициализация кольцевого буфера
        RingClear(&modH->xBufferRX);
        
        // Добавляем обработчик в массив
        mHandlers[numberHandlers] = modH;
        numberHandlers++;
    }
}

// Функция для обработки входящих данных
void Modbus_Process(modbusHandler_t *modH) {
    uint8_t u8Buffer[MAX_BUFFER];
    uint8_t u8Count = RingGetAllBytes(&modH->xBufferRX, u8Buffer);
    
    if (u8Count > 0) {
        // Обработка входящего пакета
        modH->u8BufferSize = u8Count;
        for (uint8_t i = 0; i < u8Count; i++) {
            modH->u8Buffer[i] = u8Buffer[i];
        }
        
        // Обработка функции
        uint8_t u8Function = modH->u8Buffer[FUNC];
        switch (u8Function) {
            case MB_FC_READ_REGISTERS:
                process_FC3(modH);
                break;
            case MB_FC_WRITE_MULTIPLE_REGISTERS:
                process_FC16(modH);
                break;
            default:
                // Неподдерживаемая функция
                modH->u8Buffer[FUNC] |= 0x80;
                modH->u8Buffer[2] = EXC_FUNC_CODE;
                modH->u8BufferSize = 3;
                break;
        }
        
        // Отправка ответа
        sendTxBuffer(modH);
    }
} 