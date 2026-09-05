#pragma once

// =============================================================================
// app_state.h  –  geteilter Zustand zwischen den Tasks
//
// Die Tasks laufen parallel (auf zwei CPU-Kernen). Damit sie miteinander
// reden können, teilen sie sich diese globalen Variablen. Vorsicht: Gemeinsame
// Daten brauchen Schutz – deshalb sind die Modus-Flags `volatile` und die
// Nachrichten laufen über Queues (die FreeRTOS-threadsicher sind).
// =============================================================================

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>    // QueueHandle_t
#include <lwip/sockets.h>      // sockaddr_storage, socklen_t

// Die Zustände der Netzwerk-Zustandsmaschine (siehe connection_task.cpp):
//   WIFI_CONNECT  ->  WLAN verbinden
//   WAIT_FOR_IP6  ->  auf IPv6-Adresse warten (nur Durchgangszustand)
//   DNS_RESOLVE   ->  Hostname auflösen
//   TCP_CONNECT   ->  TCP-Verbindung aufbauen
//   RUNNING       ->  normaler Betrieb (senden/empfangen)
enum ConnectionState {
  WIFI_CONNECT,
  WAIT_FOR_IP6,
  DNS_RESOLVE,
  TCP_CONNECT,
  RUNNING
};

// ------------------------- geteilter Zustand -------------------------
// Diese Variablen werden von mehreren Tasks parallel verwendet.

extern ConnectionState state;  // aktueller Zustand der Zustandsmaschine

extern struct sockaddr_storage server_addr;  // aufgelöste Server-Adresse
extern socklen_t server_addr_len;            // Länge davon

// Modus-Flags (vom Drehschalter gesetzt, `volatile` wegen Task-Zugriff).
//   true = Modus aktiv (Drehschalter steht auf LOW).
extern volatile bool NO_SOUND_MODE;      // Ton aus
extern volatile bool NO_PRINTER_MODE;    // Drucker aus
extern volatile bool SELF_CHECK_MODE;    // Selbsttest (ohne Netzwerk)
extern volatile bool SERVER_CHECK_MODE;  // Pakete zum Server zurückschicken
extern volatile bool RICK_ROLL_MODE;     // Rick Roll (Spaß-Modus)

// Kommunikationskanäle zwischen den Tasks.
// Es werden Zeiger auf Package-Objekte durchgereicht (Inhalt liegt im Heap).
extern QueueHandle_t sendQueue;      // InputTask    -> Netzwerk (TCP)
extern QueueHandle_t playbackQueue;  // Netzwerk     -> PlaybackTask
extern QueueHandle_t printQueue;     // PlaybackTask -> PrintTask

// TCP-Socket (Dateideskriptor, -1 = kein Socket offen).
extern int sock;

