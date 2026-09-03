#include "app_state.h"

ConnectionState state = WIFI_CONNECT;

struct sockaddr_storage server_addr;
socklen_t server_addr_len;

volatile bool NO_SOUND_MODE = false;
volatile bool NO_PRINTER_MODE = false;
volatile bool SELF_CHECK_MODE = false;
volatile bool SERVER_CHECK_MODE = false;
volatile bool RICK_ROLL_MODE = false;

QueueHandle_t sendQueue;
QueueHandle_t playbackQueue;
QueueHandle_t printQueue;

int sock = -1;

HardwareSerial printer(2);  // UART2 (GPIO17 TX, GPIO16 RX)
