// =============================================================================
// app_state.cpp  –  Definition der globalen (geteilten) Variablen
//
// Hier werden die in app_state.h deklarierten Variablen mit ihren Startwerten
// angelegt. Mehrere Tasks greifen parallel darauf zu (siehe app_state.h).
// =============================================================================

#include "app_state.h"

ConnectionState state = WIFI_CONNECT;  // Start: zuerst WLAN verbinden

struct sockaddr_storage server_addr;   // Zieladresse (wird per DNS gefüllt)
socklen_t server_addr_len;

// Modus-Flags: Standardmäßig alles aus (false). Der CheckerTask liest die
// Drehschalter ein und setzt sie auf true, wenn der jeweilige Modus aktiv ist.
volatile bool NO_SOUND_MODE = false;
volatile bool NO_PRINTER_MODE = false;
volatile bool SELF_CHECK_MODE = false;
volatile bool SERVER_CHECK_MODE = false;
volatile bool RICK_ROLL_MODE = false;

// Queues: werden in main.cpp mit xQueueCreate() angelegt und hier befüllt.
QueueHandle_t sendQueue;
QueueHandle_t playbackQueue;
QueueHandle_t printQueue;

int sock = -1;  // -1 = noch kein TCP-Socket offen

