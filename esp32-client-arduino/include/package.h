#pragma once

// =============================================================================
// package.h  –  Das Morse-Paket (Datenstruktur)
//
// Ein "Package" ist die Einheit, die zwischen den Tasks (über Queues) und
// über das Netzwerk (zum/vom Server) wandert. Es besteht aus einem Status
// und einer Folge von Bytes, wobei jedes Byte 8 Morse-Abtastungen enthält
// (Bit gesetzt = Taste gedrückt / Ton an).
// =============================================================================

#include <cstdint>   // uint8_t, uint16_t, uint64_t
#include <string>    // std::string
#include <vector>                 // std::vector

#include <freertos/FreeRTOS.h>   // BaseType_t, pdPASS (für Queue-API)
#include <freertos/queue.h>      // QueueHandle_t, xQueueSend, xQueueReceive

// status-Feld eines Pakets:
//   1 -> normale Morse-Nachricht (Broadcast)
//   2 -> Bestätigung (je Richtung: Zustell-ACK bzw. Annahme-ACK)
//   3 -> test (Server sendet zurück statt zu broadcasten)
//   4 -> register (Payload = 6-Byte-MAC)
struct Package {
  uint8_t status = 0;
  uint64_t msg_id = 0;                // je Richtung: client_msg_id oder server_msg_id
  uint16_t size = 0;                  // Anzahl Bytes; reicht für ~87 Minuten Morse
  std::vector<uint8_t> payload;       // die eigentlichen Morse-Daten (dynamisch)
};

// std::vector<> liegt im Heap. Deshalb werden Zeiger (Package*) durch die
// Queue geschoben und der Empfänger macht eine Kopie (Deep Copy).
bool putPackageIntoQueue(QueueHandle_t queue, const Package& package);
bool getPackageFromQueue(QueueHandle_t queue, Package& package);

// Debug-Hilfe: Paket als 0/1-String ausgeben (z. B. "11110000...").
std::string packageToText(const Package& package);

