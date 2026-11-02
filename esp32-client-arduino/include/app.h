#pragma once

#include <freertos/FreeRTOS.h> // muss vor queue.h included werden
#include <freertos/queue.h>     // QueueHandle_t

// Zustände der Netzwerk-Zustandsmaschine
enum ConnectionState { WIFI_CONNECT, TCP_CONNECT, RUNNING };

// Settings Flags (Kippschalter)
extern volatile bool AUDIO_MODE;
extern volatile bool PRINTER_MODE;
extern volatile bool TEST_MODE;
extern volatile bool HOTSPOT_MODE;

// Queues (Kommunikationskanäle zwischen den Tasks)
extern QueueHandle_t sendQueue;     // input_task    -> network_task
extern QueueHandle_t playbackQueue; // network_task  -> playback_task
extern QueueHandle_t printQueue;    // playback_task -> print_task


// sonstige geteilte Zustände
extern ConnectionState state;
extern volatile bool isRecording;
extern volatile bool isPlaying;
extern volatile bool newWifiCredentials;

// Tasks - werden jeweils in eigener .cpp definiert
void ConnectionTask(void* pvParameters);
void InputTask(void* pvParameters);
void PlaybackTask(void* pvParameters);
void CheckerTask(void* pvParameters);
void PrintTask(void* pvParameters);