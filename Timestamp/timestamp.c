#include "timestamp.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "ModbusRegisters.h"

// Константы для точной задержки
#define DELAY_STEP_US 100     // Шаг задержки 0.1 мс
#define DURATION_STEP_US 10   // Шаг длительности 0.01 мс (10 мкс)
#define MIN_DELAY_US 0        // Минимальная задержка
#define MAX_DELAY_US 200000   // Максимальная задержка 200 мс
#define MIN_DURATION_US 18000 // Минимальная длительность импульса 18 мс
#define MAX_DURATION_US 22000 // Максимальная длительность импульса 22 мс
#define MAX_ZERO_DELAY_US 5   // Максимальная задержка при нулевом значении
#define GPIO_SWITCH_TIME_US 2 // Примерное время переключения GPIO

// Глобальные переменные
osSemaphoreId_t pulseSemaphore = NULL;
PulseParams_t pulseParams = {
    .delay_us = 0,          // По умолчанию задержка 0
    .duration_us = 20000    // По умолчанию длительность 20 мс
};
volatile uint32_t debug_reg_pulse_lenght = 0;
volatile uint32_t debug_calculated_duration_us = 0;
volatile uint32_t debug_delay_us_input = 0;

// Инициализация таймера для микросекундных задержек
void TIM2_Init_Delay(void) {
    // Настройка таймера на максимальную точность
    __HAL_TIM_SET_PRESCALER(&htim2, 89);  // 90MHz / 90 = 1MHz (1 мкс на тик)
    __HAL_TIM_SET_AUTORELOAD(&htim2, 0xFFFF);
    
    // Устанавливаем максимальный приоритет для таймера
    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
    
    HAL_TIM_Base_Start(&htim2);
}

// Функция задержки в микросекундах с высокой точностью
static void delay_us(uint32_t us) {
    if (us == 0) {
        // Для нулевой задержки используем минимальное время
        __HAL_TIM_SET_COUNTER(&htim2, 0);
        while (__HAL_TIM_GET_COUNTER(&htim2) < 5); // Гарантированная задержка 5 мкс
        return;
    }
    
    // Для остальных значений используем точную задержку
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    while (__HAL_TIM_GET_COUNTER(&htim2) < us);
}

// Инициализация пина датчика
void Sensor_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // Включаем тактирование порта A
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    // Настройка PA15 как вход с подтяжкой
    GPIO_InitStruct.Pin = GPIO_PIN_15;
    // Выбор фронта в зависимости от значения регистра
    if (holdingRegisters[SP_Front_Type - 2000] == 0) {
        GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING; // восходящий фронт
    } else {
        GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING; // падающий фронт
    }
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // Установка максимального приоритета прерывания
    //HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
    //HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

// Функция округления до ближайшего шага
static uint32_t round_to_step(uint32_t value, uint32_t step) {
    return ((value + step/2) / step) * step;
}

// Функция проверки и корректировки параметров
void validate_pulse_params(PulseParams_t* params) {
    // Получаем значения из регистров Modbus
    uint32_t delay = holdingRegisters[SP_Delay_Before - 2000] * 100; // Convert from 0.1ms to us
    uint32_t duration = holdingRegisters[SP_Pulse_Lenght - 2000] * 10; // Convert from 0.01ms to us

    // Проверяем и ограничиваем задержку
    if (delay > MAX_DELAY_US) {
        delay = MAX_DELAY_US;
    }
    params->delay_us = round_to_step(delay, DELAY_STEP_US);

    // Проверяем и ограничиваем длительность
    if (duration < MIN_DURATION_US) {
        duration = MIN_DURATION_US;
    } else if (duration > MAX_DURATION_US) {
        duration = MAX_DURATION_US;
    }
    params->duration_us = round_to_step(duration, DURATION_STEP_US);

    // Обновляем регистры обратной связи
    inputRegisters[FBK_Delay_Before - 1000] = params->delay_us / 100; // Convert back to 0.1ms
    inputRegisters[FBK_Pulse_Lenght - 1000] = params->duration_us / 10; // Convert back to 0.01ms
}

// Неблокирующая генерация импульса
void GeneratePulse(void) {
    // Проверяем и корректируем параметры
    validate_pulse_params(&pulseParams);

    // Устанавливаем флаг активного импульса
    inputRegisters[FBK_Pulse_On - 1000] = 1;

    // Генерация импульса с высокой точностью
    if (pulseParams.delay_us == 0) {
        // Для нулевой задержки используем минимальное время
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
        delay_us(MAX_ZERO_DELAY_US); // Максимальная задержка 5 мкс
    } else {
        // Для ненулевой задержки используем установленное значение
        delay_us(pulseParams.delay_us);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    }

    // Отладочная информация
    debug_reg_pulse_lenght = holdingRegisters[SP_Pulse_Lenght - 2000];
    debug_calculated_duration_us = pulseParams.duration_us;
    debug_delay_us_input = pulseParams.duration_us;

    // Генерируем импульс заданной длительности
    delay_us(pulseParams.duration_us);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

    // Обновляем счетчик с проверкой переполнения
    if (holdingRegisters[SP_Pos_Count - 2000] < UINT16_MAX) {
        holdingRegisters[SP_Pos_Count - 2000]++;
    }
    inputRegisters[FBK_Pulse_Count - 1000] = holdingRegisters[SP_Pos_Count - 2000];

    // Сбрасываем флаг активного импульса
    inputRegisters[FBK_Pulse_On - 1000] = 0;
}

// Задача обработки датчика
void SensorTask(void *argument) {
    (void)argument;
    TIM2_Init_Delay();
    Sensor_Init();

    // Инициализация значений по умолчанию
    holdingRegisters[SP_Delay_Before - 2000] = 0;     // Задержка 0 мс
    holdingRegisters[SP_Pulse_Lenght - 2000] = 2000;  // Длительность 20 мс
    holdingRegisters[SP_Front_Type - 2000] = 0;       // Восходящий фронт

    // Update feedback registers
    inputRegisters[FBK_Delay_Before - 1000] = holdingRegisters[SP_Delay_Before - 2000];
    inputRegisters[FBK_Pulse_Lenght - 1000] = holdingRegisters[SP_Pulse_Lenght - 2000];
    inputRegisters[FBK_Front_Type - 1000] = holdingRegisters[SP_Front_Type - 2000];

    for (;;) {
        // Задача теперь только обновляет параметры
        validate_pulse_params(&pulseParams);
        osDelay(10); // Обновляем параметры каждые 10 мс
    }
}