#include "ModbusTCPServer.h"
#include "Modbus.h"
#include "ModbusConfig.h"
#include "ModbusRegisters.h"
#include "lwip/tcp.h"
#include "lwip/ip.h"
#include "lwip/init.h"
#include "lwip/netif.h"
#include "lwip/ip4.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

// Объявления функций обработки Modbus
extern int8_t process_FC3(modbusHandler_t *modH);
extern int8_t process_FC4(modbusHandler_t *modH);
extern int8_t process_FC16(modbusHandler_t *modH);

// Добавляем константы для таймаутов
#define TCP_CONNECTION_TIMEOUT_MS 30000  // 30 секунд таймаут соединения
#define TCP_KEEPALIVE_INTERVAL_MS 5000   // 5 секунд интервал keepalive
#define TCP_MAX_RETRIES 3               // Максимальное количество попыток переподключения

// Структура для хранения информации о TCP-соединении
typedef struct {
    struct tcp_pcb *pcb;
    modbusHandler_t *modH;
    uint32_t lastActivity;
    uint32_t lastKeepalive;
    uint8_t retryCount;
    uint8_t buffer[MAX_BUFFER];
    uint16_t bufferSize;
    bool isConnected;
} tcpConnection_t;

// Объявления функций TCP
static err_t tcpAccept(void *arg, struct tcp_pcb *newpcb, err_t err);
static err_t tcpRecv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err);
static err_t tcpSent(void *arg, struct tcp_pcb *pcb, u16_t len);
static err_t tcpPoll(void *arg, struct tcp_pcb *pcb);
static void tcpError(void *arg, err_t err);
static void handleConnectionClose(tcpConnection_t *conn);
static void tryReconnect(tcpConnection_t *conn);

// Массив активных соединений
static tcpConnection_t connections[NUMBERTCPCONN];
static uint8_t activeConnections = 0;

// Функция для поиска свободного слота в массиве соединений
static int8_t findFreeConnectionSlot(void) {
    for (uint8_t i = 0; i < NUMBERTCPCONN; i++) {
        if (connections[i].pcb == NULL) {
            return i;
        }
    }
    return -1;
}

// Функция для поиска соединения по PCB
static int8_t findConnectionByPCB(struct tcp_pcb *pcb) {
    for (uint8_t i = 0; i < NUMBERTCPCONN; i++) {
        if (connections[i].pcb == pcb) {
            return i;
        }
    }
    return -1;
}

// Обработчик принятия нового соединения
static err_t tcpAccept(void *arg, struct tcp_pcb *newpcb, err_t err) {
    modbusHandler_t *modH = (modbusHandler_t *)arg;
    if (err != ERR_OK || newpcb == NULL || modH == NULL) {
        return ERR_VAL;
    }

    if (activeConnections >= NUMBERTCPCONN) {
        tcp_close(newpcb);
        return ERR_CONN;
    }

    int8_t slot = findFreeConnectionSlot();
    if (slot < 0) {
        tcp_close(newpcb);
        return ERR_CONN;
    }

    // Инициализация нового соединения
    connections[slot].pcb = newpcb;
    connections[slot].modH = modH;
    connections[slot].lastActivity = xTaskGetTickCount();
    connections[slot].lastKeepalive = xTaskGetTickCount();
    connections[slot].bufferSize = 0;
    connections[slot].isConnected = true;
    connections[slot].retryCount = 0;

    // Сохраняем соединение в обработчике
    modH->newconns[slot].conn = (struct netconn *)newpcb;
    modH->newconns[slot].aging = 0;
    modH->newconnIndex = slot;

    // Настраиваем обработчики
    tcp_arg(newpcb, &connections[slot]);
    tcp_recv(newpcb, tcpRecv);
    tcp_sent(newpcb, tcpSent);
    tcp_poll(newpcb, tcpPoll, 1);
    tcp_err(newpcb, tcpError);

    activeConnections++;
    return ERR_OK;
}

// Обработчик получения данных
static err_t tcpRecv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err) {
    tcpConnection_t *conn = (tcpConnection_t *)arg;
    
    if (err != ERR_OK || p == NULL) {
        if (err == ERR_CLSD) {
            handleConnectionClose(conn);
        }
        return ERR_VAL;
    }

    conn->lastActivity = xTaskGetTickCount();

    if (p->tot_len > MAX_BUFFER) {
        pbuf_free(p);
        return ERR_MEM;
    }

    pbuf_copy_partial(p, conn->buffer, p->tot_len, 0);
    conn->bufferSize = p->tot_len;

    pbuf_free(p);

    // Обработка Modbus-запроса
    for (uint8_t i = 0; i < MAX_M_HANDLERS; i++) {
        if (mHandlers[i] != NULL) {
            memcpy(mHandlers[i]->u8Buffer, conn->buffer, conn->bufferSize);
            mHandlers[i]->u8BufferSize = conn->bufferSize;
            
            switch (mHandlers[i]->u8Buffer[FUNC]) {
                case MB_FC_READ_INPUT_REGISTER:
                    process_FC4(mHandlers[i]);
                    break;
                case MB_FC_READ_REGISTERS:
                    process_FC3(mHandlers[i]);
                    break;
                case MB_FC_WRITE_MULTIPLE_REGISTERS:
                    process_FC16(mHandlers[i]);
                    break;
                default:
                    mHandlers[i]->u8Buffer[FUNC] |= 0x80;
                    mHandlers[i]->u8Buffer[2] = EXC_FUNC_CODE;
                    mHandlers[i]->u8BufferSize = 3;
                    break;
            }
            
            tcp_write(pcb, mHandlers[i]->u8Buffer, mHandlers[i]->u8BufferSize, 1);
            tcp_output(pcb);
            break;
        }
    }

    return ERR_OK;
}

// Обработчик отправки данных
static err_t tcpSent(void *arg, struct tcp_pcb *pcb, u16_t len) {
    tcpConnection_t *conn = (tcpConnection_t *)arg;
    (void)pcb;
    (void)len;
    conn->lastActivity = xTaskGetTickCount();
    return ERR_OK;
}

// Обработчик опроса соединения
static err_t tcpPoll(void *arg, struct tcp_pcb *pcb) {
    tcpConnection_t *conn = (tcpConnection_t *)arg;
    uint32_t currentTime = xTaskGetTickCount();
    
    if (findConnectionByPCB(pcb) < 0) {
        return ERR_OK;
    }
    
    if ((currentTime - conn->lastActivity) > TCP_CONNECTION_TIMEOUT_MS) {
        handleConnectionClose(conn);
        return ERR_OK;
    }
    
    if ((currentTime - conn->lastKeepalive) > TCP_KEEPALIVE_INTERVAL_MS) {
        uint8_t keepaliveMsg[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        tcp_write(pcb, keepaliveMsg, sizeof(keepaliveMsg), 1);
        tcp_output(pcb);
        conn->lastKeepalive = currentTime;
    }
    
    return ERR_OK;
}

// Обработчик ошибок
static void tcpError(void *arg, err_t err) {
    tcpConnection_t *conn = (tcpConnection_t *)arg;
    if (conn != NULL) {
        if (err == ERR_CONN || err == ERR_RST) {
            tryReconnect(conn);
        } else {
            handleConnectionClose(conn);
        }
    }
}

// Функция обработки закрытия соединения
static void handleConnectionClose(tcpConnection_t *conn) {
    if (conn == NULL) {
        return;
    }

    if (conn->pcb != NULL) {
        tcp_close(conn->pcb);
        conn->pcb = NULL;
    }

    // Очищаем соединение в обработчике
    if (conn->modH != NULL) {
        for (uint8_t i = 0; i < NUMBERTCPCONN; i++) {
            if ((struct tcp_pcb *)conn->modH->newconns[i].conn == conn->pcb) {
                conn->modH->newconns[i].conn = NULL;
                conn->modH->newconns[i].aging = 0;
                break;
            }
        }
    }

    conn->isConnected = false;
    conn->retryCount = 0;
    conn->bufferSize = 0;
    activeConnections--;
}

// Функция для попытки переподключения
static void tryReconnect(tcpConnection_t *conn) {
    if (conn->retryCount < TCP_MAX_RETRIES) {
        conn->retryCount++;
        struct tcp_pcb *newpcb = tcp_new();
        if (newpcb != NULL) {
            tcp_arg(newpcb, conn);
            tcp_recv(newpcb, tcpRecv);
            tcp_sent(newpcb, tcpSent);
            tcp_poll(newpcb, tcpPoll, 1);
            tcp_err(newpcb, tcpError);
            conn->pcb = newpcb;
        }
    } else {
        handleConnectionClose(conn);
    }
}

// Инициализация TCP-сервера
err_t Modbus_InitTCPServer(modbusHandler_t *modH) {
    if (modH == NULL) {
        return ERR_ARG;
    }

    // Инициализация массива соединений
    memset(connections, 0, sizeof(connections));
    activeConnections = 0;

    // Создаем TCP PCB
    struct tcp_pcb *pcb = tcp_new();
    if (pcb == NULL) {
        return ERR_MEM;
    }

    // Привязываем к порту
    err_t err = tcp_bind(pcb, IP_ADDR_ANY, modH->uTcpPort);
    if (err != ERR_OK) {
        tcp_close(pcb);
        return err;
    }

    // Переводим в режим прослушивания
    pcb = tcp_listen(pcb);
    if (pcb == NULL) {
        return ERR_MEM;
    }

    // Сохраняем PCB в обработчике
    modH->conn = (struct netconn *)pcb;
    modH->xTypeHW = TCP_HW;
    modH->uModbusType = MB_SLAVE;

    // Настраиваем обработчики
    tcp_arg(pcb, modH);
    tcp_accept(pcb, tcpAccept);

    return ERR_OK;
} 