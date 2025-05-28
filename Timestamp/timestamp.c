#include "timestamp.h"
#include "tim.h"
#include "gpio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "ModbusRegisters.h"
#include "iwdg.h"

// Константы для точной задержки
#define DELAY_STEP_US 100     // Шаг задержки 0.1 мс
#define DURATION_STEP_US 10   // Шаг длительности 0.01 мс
#define MIN_DELAY_US 0        // Минимальная задержка
#define MAX_DELAY_US 200000   // Максимальная задержка 200 мс
#define MIN_DURATION_US 18000 // Минимальная длительность импульса 18 мс
#define MAX_DURATION_US 22000 // Максимальная длительность импульса 22 мс
#define MAX_ZERO_DELAY_US 5   // Максимальная задержка при нулевом значении

// Глобальные переменные
osSemaphoreId_t pulseSemaphore = NULL;
PulseParams_t pulseParams = {
    .delay_us = 0,          // По умолчанию задержка 0
    .duration_us = 20000    // По умолчанию длительность 20 мс
};

// Инициализация таймера для микросекундных задержек
void TIM2_Init_Delay(void) {
    // Настройка таймера на максимальную точность
    __HAL_TIM_SET_PRESCALER(&htim2, 84 - 1);  // Для 1 МГц (84 МГц / 84)
    __HAL_TIM_SET_AUTORELOAD(&htim2, 0xFFFF);
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
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING; // восходящий фронт
    //GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING; // падающий фронт
    //GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING; // Обработка обоих фронтов
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // Установка приоритета прерывания
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

// Функция округления до ближайшего шага
static uint32_t round_to_step(uint32_t value, uint32_t step) {
    return ((value + step/2) / step) * step;
}

// Функция проверки и корректировки параметров
static void validate_pulse_params(PulseParams_t* params) {
    // Проверка и округление задержки
    if (params->delay_us > MAX_DELAY_US) {
        params->delay_us = MAX_DELAY_US;
    }
    params->delay_us = round_to_step(params->delay_us, DELAY_STEP_US);
    
    // Проверка и округление длительности
    if (params->duration_us < MIN_DURATION_US) {
        params->duration_us = MIN_DURATION_US;
    } else if (params->duration_us > MAX_DURATION_US) {
        params->duration_us = MAX_DURATION_US;
    }
    params->duration_us = round_to_step(params->duration_us, DURATION_STEP_US);
}

// Неблокирующая генерация импульса
void GeneratePulse(void) {
    // Проверка и корректировка параметров с учетом шага
    validate_pulse_params(&pulseParams);

    // Генерация импульса с высокой точностью
    if (pulseParams.delay_us == 0) {
        // При нулевой задержке используем минимальное время
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
        delay_us(MAX_ZERO_DELAY_US); // Максимальная задержка 5 мкс
    } else {
        // При ненулевой задержке используем заданное значение
        delay_us(pulseParams.delay_us);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    }
    
    delay_us(pulseParams.duration_us);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

    // Обновление счетчика с проверкой переполнения
    if (holdingRegisters[SP_Pos_Count - 2000] < UINT16_MAX) {
        holdingRegisters[SP_Pos_Count - 2000]++;
    }
    inputRegisters[FBK_Pulse_Count - 1000] = holdingRegisters[SP_Pos_Count - 2000];
    
    // Сброс watchdog после критической операции
    HAL_IWDG_Refresh(&hiwdg);
}

// Задача обработки датчика
void SensorTask(void *argument) {
    (void)argument;
    TIM2_Init_Delay();
    Sensor_Init();
    pulseParams.delay_us = 1000;
    pulseParams.duration_us = 21000;
    holdingRegisters[SP_Front_Type - 2000] = 0;
    HAL_IWDG_Refresh(&hiwdg);
    for (;;) {
        size_t watermark = uxTaskGetStackHighWaterMark(NULL);
        if (xSemaphoreTake(pulseSemaphore, portMAX_DELAY) == pdTRUE) {
            GeneratePulse();
            inputRegisters[FBK_Pulse_On - 1000] = 0;
            HAL_IWDG_Refresh(&hiwdg);
        }
    }
}