// =============================================================================
// main.cpp  –  Einstiegspunkt
//
// Hier wird in dieser Reihenfolge alles hochgefahren:
//
//   1. Serial     – Konsole (115200 Baud) für die Log-Ausgabe
//   2. WLAN-Daten – gespeicherte Zugangsdaten laden (Preferences/NVS)
//   3. WLAN       – WiFi initialisieren (WiFiMulti + AP-Liste)
//   4. Hardware   – GPIOs (LED, Piezo, MOSFET, Taste, Kippschalter)
//   5. Drucker    – Thermodrucker (Serial1/UART1)
//   6. Tasks      – je eine eigene Endlosschleife (FreeRTOS)
//
// Danach läuft alles nebenläufig über FreeRTOS-Tasks weiter.
//
// =============================================================================
#include "hardware.h"
#include "log.h"
#include "storage.h"
#include "app.h"
#include "package.h"
#include "network.h"

#include <Arduino.h>
#include <cstddef>
#include <freertos/task.h>   // vTaskDelete
#include <freertos/idf_additions.h>


static const char* TAG = "main";

void setup() {
    Serial.begin(115200);

    // Version der ESP-IDF ausgeben (nur zu Info-Zwecken).
    LOGI(TAG, "ESP-IDF %s", esp_get_idf_version());
    LOGI(TAG, "Firmware %s", FIRMWARE_VERSION);

    // Persistente WLAN-Zugangsdaten aus dem NVS laden (falls schon gespeichert).
    loadWifiConfig();

    wifiInit();

    // Initialisiert alle GPIOs als Ein- oder Ausgänge und schaltet den MOSFET ein.
    hardwareInit();

    printerInit();

    // Alle Tasks in einer Tabelle beschreiben: { Funktion, Name, Priorität }.
    // Priorität: höher = wichtiger (bekommt bei Konkurrenz zuerst CPU-Zeit).
    struct TaskDef {
        TaskFunction_t func;   // die Task-Funktion (Signatur: void(void*))
        const char* name;      // Name (nur für Debug-Ausgaben)
        int priority;          // Priorität (0..N, höher = wichtiger)
    };
    constexpr int TASK_STACK_SIZE = 4096;  // Stack pro Task in Bytes
    const TaskDef tasks[] = {
        { ConnectionTask, "Check WiFi TCP", 1 },  // Netzwerk-Zustandsmaschine
        { InputTask,      "Input Task",     1 },  // Morse-Taste abtasten
        { PlaybackTask,   "Output Task",    1 },  // Ton + LED
        { CheckerTask,    "Checker Task",   2 },  // Drehschalter + Status-LED + MOSFET
        { PrintTask,      "Print Task",     2 },  // Thermodrucker
    };

    // Der ESP32-C5 ist ein Single-Core-RISC-V: alle Tasks laufen auf dem
    // einzigen Kern. Deshalb xTaskCreate() statt xTaskCreatePinnedToCore().
    for (const TaskDef& t : tasks)
        xTaskCreate(t.func, t.name, TASK_STACK_SIZE, NULL, t.priority, NULL);
}

void loop() {
    vTaskDelete(NULL);
}