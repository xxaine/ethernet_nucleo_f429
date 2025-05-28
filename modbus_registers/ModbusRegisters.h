#pragma once
#include "Modbus.h"  // Должен быть первым, так как определяет modbusHandler_t
#include <stdint.h>

// Объявления переменных
extern modbusHandler_t ModbusH;

// Input Registers (только чтение, адреса 1000-1019)
typedef enum {
    FBK_Delay_Before = 1000,
    FBK_Delay_Count = 1001,
    FBK_Pos_Count = 1002,
    FBK_Pos_Count_Max = 1003,
    FBK_Pos = 1004,
    FBK_Pulse_Lenght = 1005,
    FBK_Pulse_Count = 1006,
    FBK_Front_Type = 1007,
    FBK_Pulse_On = 1008,
    FBK_Power_27_V = 1009,
    FBK_POS_Set = 1010,
    FBK_Reserve_11 = 1011,
    FBK_Reserve_12 = 1012,
    FBK_Reserve_13 = 1013,
    FBK_Reserve_14 = 1014,
    FBK_Reserve_15 = 1015,
    FBK_Reserve_16 = 1016,
    FBK_Reserve_17 = 1017,
    FBK_Reserve_18 = 1018,
    FBK_Reserve_19 = 1019
} InputRegisters;

// Holding Registers (чтение/запись, адреса 2000-2019)
typedef enum {
    SP_Delay_Before = 2000,
    SP_reserve_1 = 2001,
    SP_Pos_Count = 2002,
    SP_Pos_Count_Max = 2003,
    SP_reserve_4 = 2004,
    SP_Pulse_Lenght = 2005,
    SP_reserve_6 = 2006,
    SP_Front_Type = 2007,
    SP_Pulse_On = 2008,
    SP_Power_27_V = 2009,
    SP_POS_Set = 2010,
    SP_reserve_11 = 2011,
    SP_reserve_12 = 2012,
    SP_reserve_13 = 2013,
    SP_reserve_14 = 2014,
    SP_reserve_15 = 2015,
    SP_reserve_16 = 2016,
    SP_reserve_17 = 2017,
    SP_reserve_18 = 2018,
    SP_reserve_19 = 2019
} HoldingRegisters;

// Размеры массивов
#define INPUT_REGISTERS_COUNT  20
#define HOLDING_REGISTERS_COUNT 20

// Глобальные массивы регистров
extern uint16_t inputRegisters[INPUT_REGISTERS_COUNT];
extern uint16_t holdingRegisters[HOLDING_REGISTERS_COUNT];

// Глобальные переменые для энкодера
extern int32_t encoderPosition;
extern int32_t encoderPulsesPerRevolution;
extern int32_t encoderSpeed;

// Функции для работы с регистрами
void ModbusRegisters_Init(void);
uint16_t Modbus_GetInputRegister(InputRegisters reg);
void Modbus_SetHoldingRegister(HoldingRegisters reg, uint16_t value);