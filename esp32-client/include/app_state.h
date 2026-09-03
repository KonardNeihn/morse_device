#pragma once

#include <Arduino.h>          // HardwareSerial
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>    // QueueHandle_t
#include <lwip/sockets.h>      // sockaddr_storage, socklen_t

// Verbindungszustand der Netzwerk-Zustandsmaschine.
enum ConnectionState {
  WIFI_CONNECT,
  WAIT_FOR_IP6,
  DNS_RESOLVE,
  TCP_CONNECT,
  RUNNING
};

// ------------------------- geteilter Zustand -------------------------
// Diese Variablen werden von mehreren Tasks parallel verwendet.

extern ConnectionState state;

extern struct sockaddr_storage server_addr;
extern socklen_t server_addr_len;

// Modus-Flags (von den Drehschaltern gesetzt, volatile wegen Task-Zugriff).
extern volatile bool NO_SOUND_MODE;
extern volatile bool NO_PRINTER_MODE;
extern volatile bool SELF_CHECK_MODE;
extern volatile bool SERVER_CHECK_MODE;
extern volatile bool RICK_ROLL_MODE;

// Kommunikationskanäle zwischen den Tasks.
extern QueueHandle_t sendQueue;      // InputTask    -> Netzwerk (TCP)
extern QueueHandle_t playbackQueue;  // Netzwerk     -> PlaybackTask
extern QueueHandle_t printQueue;     // PlaybackTask -> PrintTask

// TCP-Socket.
extern int sock;

// Thermodrucker (UART2: GPIO17 TX, GPIO16 RX).
extern HardwareSerial printer;
