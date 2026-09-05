#pragma once

// =============================================================================
// package.h  –  Das Morse-Paket (Datenstruktur)
//
// Ein "Package" ist die Einheit, die zwischen den Tasks (über Queues) und
// über das Netzwerk (zum/vom Server) wandert. Es besteht aus einem Status
// und einer Folge von Bytes, wobei jedes Byte 8 Morse-Abtastungen enthält
// (Bit gesetzt = Taste gedrückt / Ton an).
// =============================================================================

#include <string>
#include <vector>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

// status-Feld eines Pakets:
//   0 -> keep alive (Verbindung am Leben halten)
//   1 -> normales Morse-Paket (an den Server)
//   2 -> Bestätigung "Paket empfangen"
//   3 -> server_check_mode (Paket wird zum Server zurückgeschickt)
struct Package {
  uint8_t status = 0;
  uint16_t size = 0;                  // Anzahl Bytes; reicht für ~87 Minuten Morse
  std::vector<uint8_t> payload;       // die eigentlichen Morse-Daten (dynamisch)
};

// std::vector<> liegt im Heap. Deshalb werden Zeiger (Package*) durch die
// Queue geschoben und der Empfänger macht eine Kopie (Deep Copy).
bool putPackageIntoQueue(QueueHandle_t queue, const Package& package);
bool getPackageFromQueue(QueueHandle_t queue, Package& package);

// Debug-Hilfe: Paket als 0/1-String ausgeben (z. B. "11110000...").
std::string packageToText(const Package& package);

