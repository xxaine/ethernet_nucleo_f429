#include "Modbus.h"

void RingAdd(modbusRingBuffer_t *xRingBuffer, uint8_t u8Val) {
    if (xRingBuffer->u8available < MAX_BUFFER - 1) {
        xRingBuffer->uxBuffer[xRingBuffer->u8end] = u8Val;
        xRingBuffer->u8end = (xRingBuffer->u8end + 1) % MAX_BUFFER;
        xRingBuffer->u8available++;
    } else {
        xRingBuffer->overflow = true;
    }
}

uint8_t RingGetAllBytes(modbusRingBuffer_t *xRingBuffer, uint8_t *buffer) {
    uint8_t count = 0;
    while (xRingBuffer->u8available > 0) {
        buffer[count++] = xRingBuffer->uxBuffer[xRingBuffer->u8start];
        xRingBuffer->u8start = (xRingBuffer->u8start + 1) % MAX_BUFFER;
        xRingBuffer->u8available--;
    }
    return count;
}

uint8_t RingGetNBytes(modbusRingBuffer_t *xRingBuffer, uint8_t *buffer, uint8_t uNumber) {
    uint8_t count = 0;
    while (xRingBuffer->u8available > 0 && count < uNumber) {
        buffer[count++] = xRingBuffer->uxBuffer[xRingBuffer->u8start];
        xRingBuffer->u8start = (xRingBuffer->u8start + 1) % MAX_BUFFER;
        xRingBuffer->u8available--;
    }
    return count;
}

uint8_t RingCountBytes(modbusRingBuffer_t *xRingBuffer) {
    return xRingBuffer->u8available;
}

void RingClear(modbusRingBuffer_t *xRingBuffer) {
    xRingBuffer->u8start = 0;
    xRingBuffer->u8end = 0;
    xRingBuffer->u8available = 0;
    xRingBuffer->overflow = false;
} 