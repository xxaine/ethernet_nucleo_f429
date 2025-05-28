#ifndef __MODBUS_TCP_SERVER_H
#define __MODBUS_TCP_SERVER_H

#include "Modbus.h"
#include "lwip/err.h"

#define MODBUS_TCP_PORT 502

err_t Modbus_InitTCPServer(modbusHandler_t *modH);

#endif /* __MODBUS_TCP_SERVER_H */ 