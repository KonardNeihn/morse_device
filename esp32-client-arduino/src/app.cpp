#include "app.h"
#include "config.h"
#include "package.h"
#include "freertos/idf_additions.h"

// Settings Flags (Kippschalter)
volatile bool AUDIO_MODE = false;
volatile bool PRINTER_MODE = false;
volatile bool TEST_MODE = false;
volatile bool HOTSPOT_MODE = false;

// Queues (Kommunikationkanäle zwischen den Tasks)
QueueHandle_t sendQueue     = xQueueCreate(QUEUE_SIZE, sizeof(Package*));  // input_task/printer_task -> connection_task
QueueHandle_t playbackQueue = xQueueCreate(QUEUE_SIZE, sizeof(Package*));  // connection_task         -> playback_task
QueueHandle_t printQueue    = xQueueCreate(QUEUE_SIZE, sizeof(Package*));  // playback_task           -> printer_task

// sonstige geteilte Zustände
ConnectionState state = WIFI_CONNECT;
volatile bool isRecording = false;
volatile bool isPlaying = false;
volatile bool newWifiCredentials = false;