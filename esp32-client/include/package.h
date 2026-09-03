#pragma once

#include <string>
#include <vector>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

// Ein Morse-Paket, wie es über die Queues zwischen den Tasks wandert.
//
// status:
//   0 -> keep alive
//   1 -> normales Paket
//   2 -> Bestätigung "Paket empfangen"
//   3 -> server_check_mode (wird zurückgeschickt)
struct Package {
  uint8_t status = 0;
  uint16_t size = 0;                  // reicht für 87 Minuten
  std::vector<uint8_t> payload;       // dynamische Größe
};

// std::vector<> liegt im Heap, deshalb werden Zeiger durch die Queue geschoben.
bool putPackageIntoQueue(QueueHandle_t queue, const Package& package);
bool getPackageFromQueue(QueueHandle_t queue, Package& package);

// Debug-Hilfe: Paket als 0/1-String ausgeben.
std::string packageToText(const Package& package);
