#ifndef __TIMESTAMP_H
#define __TIMESTAMP_H

#include "main.h"

// Структура для хранения параметров импульса
typedef struct {
    uint32_t delay_us;      // Задержка в микросекундах
    uint32_t duration_us;   // Длительность в микросекундах
} PulseParams_t;

// Объявления функций
void TIM2_Init_Delay(void);
void Sensor_Init(void);
void GeneratePulse(void);
void SensorTask(void *argument);

// Объявления внешних переменных
extern osSemaphoreId_t pulseSemaphore;
extern PulseParams_t pulseParams;

#endif /* __TIMESTAMP_H */
