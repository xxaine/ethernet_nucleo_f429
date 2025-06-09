/*
 * ModbusConfig.h
 *
 *  Created on: Apr 28, 2021
 *      Author: Alejandro Mera
 *
 *  This is a template for the Modbus library configuration.
 *  Every project needs a tailored copy of this file renamed to ModbusConfig.h, and added to the include path.
 */

#ifndef THIRD_PARTY_MODBUS_LIB_CONFIG_MODBUSCONFIG_H_
#define THIRD_PARTY_MODBUS_LIB_CONFIG_MODBUSCONFIG_H_



/* Uncomment the following line to enable support for Modbus RTU over USB CDC profile. Only tested for BluePill f103 board. */
//#define ENABLE_USB_CDC 1

/* Uncomment the following line to enable support for Modbus TCP. Only tested for Nucleo144-F429ZI. */
#define ENABLE_TCP 1

/* Uncomment the following line to enable support for Modbus RTU USART DMA mode. Only tested for Nucleo144-F429ZI.  */
//#define ENABLE_USART_DMA 1


#define T35  5              // Timer T35 period (in ticks) for end frame detection.
#define MAX_BUFFER  128     // Maximum size for the communication buffer in bytes.
#define TIMEOUT_MODBUS 2000 // Timeout for master query (in ticks)
#define MAX_M_HANDLERS 2    //Maximum number of modbus handlers that can work concurrently
#define MAX_TELEGRAMS 2     //Max number of Telegrams in master queue
#define MAX_REGS 20         //Maximum number of registers supported

#if ENABLE_TCP == 1
#define NUMBERTCPCONN   4   // Maximum number of simultaneous client connections, it should be equal or less than LWIP configuration
#define TCPAGINGCYCLES  10000 // Number of times the server will check for a incoming request before closing the connection for inactivity
#define RECONNECT_DELAY 1000
#define MAX_RECONNECT_ATTEMPTS 3
#define REQUEST_DELAY 2000
/* Note: the total aging time for a connection is approximately NUMBERTCPCONN*TCPAGINGCYCLES*u16timeOut ticks
*/
#endif

// Modbus RTU Configuration
#define ENABLE_RTU 0
#define RTU_BAUDRATE 9600
#define RTU_PARITY 0
#define RTU_STOPBITS 1

// Modbus Function Codes
#define READ_COILS 0x01
#define READ_DISCRETE_INPUTS 0x02
#define READ_HOLDING_REGISTERS 0x03
#define READ_INPUT_REGISTERS 0x04
#define WRITE_SINGLE_COIL 0x05
#define WRITE_SINGLE_REGISTER 0x06
#define WRITE_MULTIPLE_COILS 0x0F
#define WRITE_MULTIPLE_REGISTERS 0x10

// Modbus Address Configuration
#define MODBUS_BROADCAST_ADDRESS 0
#define MODBUS_MIN_ADDRESS 1
#define MODBUS_MAX_ADDRESS 247

// Error Codes
#define MODBUS_EXCEPTION_ILLEGAL_FUNCTION 0x01
#define MODBUS_EXCEPTION_ILLEGAL_ADDRESS 0x02
#define MODBUS_EXCEPTION_ILLEGAL_VALUE 0x03
#define MODBUS_EXCEPTION_DEVICE_FAILURE 0x04
#define MODBUS_EXCEPTION_ACKNOWLEDGE 0x05
#define MODBUS_EXCEPTION_DEVICE_BUSY 0x06
#define MODBUS_EXCEPTION_NEGATIVE_ACKNOWLEDGE 0x07
#define MODBUS_EXCEPTION_MEMORY_PARITY 0x08
#define MODBUS_EXCEPTION_GATEWAY_PATH 0x0A
#define MODBUS_EXCEPTION_GATEWAY_TARGET 0x0B

#endif /* THIRD_PARTY_MODBUS_LIB_CONFIG_MODBUSCONFIG_H_ */
