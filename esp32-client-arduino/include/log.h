#pragma once

// =============================================================================
// log.h  –  Zentrale Log-Schnittstelle des Morse-Clients
//
// Dieses Modul ist der EINZIGE Ort, über den im ganzen Projekt geloggt wird.
// Die übrigen Module inkludieren nur diese Datei und schreiben dann z. B.:
//
//     LOGI(TAG, "Verbinde mit %s", ssid);
//
// Jede Zeile sieht im Monitor so aus (farbig, mit Zeitstempel und Tag):
//
//     I (00:00:01:02.345) network    : Verbinde mit morse-server.de
//
// -----------------------------------------------------------------------------
// Warum EIGENE Makros (LOGI/LOGW/...) statt ESP_LOGI/ESP_LOGW/...?
//
// Der Arduino-ESP32-Core wird mit CONFIG_LOG_MAXIMUM_LEVEL = ERROR gebaut.
// Dadurch sind ESP_LOGI/ESP_LOGW/ESP_LOGD/ESP_LOGV dort bereits WEGKOMPILIERT –
// nur ESP_LOGE würde noch etwas ausgeben. Damit wir trotzdem Info-/Debug-Logs
// haben, setzen wir auf eine eigene, Serial-basierte Ausgabe (siehe log.cpp).
// Von ESP-IDF übernehmen wir nur die Level-Konstanten (ESP_LOG_INFO usw.).
// =============================================================================

#include <esp_log.h>    // nur für die Level-Konstanten + den Typ esp_log_level_t
#include "config.h"     // für LOG_LEVEL (Kompilierzeit-Loglevel)

// ------------------------- Die Log-Makros -----------------------------------
// Jedes Makro prüft beim Übersetzen, ob sein Level aktiviert ist. Da LOG_LEVEL
// eine Kompilierzeit-Konstante ist, faltet der Compiler die Bedingung zu
// `true`/`false` und wirft inaktive Aufrufe komplett raus (mit -Os).
//
// Parameter:
//   tag  = kurzer Modulname, z. B. "network", "hardware", "main"
//   ...  = printf-artiger Formatstring + optionale Werte
//
// Wichtig: immer mindestens den Formatstring angeben (auch ganz ohne Werte):
//   LOGI(TAG, "verbunden");     // ok
//   LOGI(TAG, "x = %d", x);     // ok
//   LOGI(TAG);                  // NICHT erlaubt (Formatstring fehlt)
//
// Das do { } while(0) macht die Makros sicher in if/else und Schleifen
// (ohne dieses Muster könnte z. B. ein if ohne Klammern auseinanderlaufen).

#define LOGI(tag, ...) do { if (ESP_LOG_INFO    <= LOG_LEVEL) logWrite(ESP_LOG_INFO,    tag, __VA_ARGS__); } while(0)
#define LOGW(tag, ...) do { if (ESP_LOG_WARN    <= LOG_LEVEL) logWrite(ESP_LOG_WARN,    tag, __VA_ARGS__); } while(0)
#define LOGE(tag, ...) do { if (ESP_LOG_ERROR   <= LOG_LEVEL) logWrite(ESP_LOG_ERROR,   tag, __VA_ARGS__); } while(0)
#define LOGD(tag, ...) do { if (ESP_LOG_DEBUG   <= LOG_LEVEL) logWrite(ESP_LOG_DEBUG,   tag, __VA_ARGS__); } while(0)
#define LOGV(tag, ...) do { if (ESP_LOG_VERBOSE <= LOG_LEVEL) logWrite(ESP_LOG_VERBOSE, tag, __VA_ARGS__); } while(0)

// ------------------------- Kernfunktion --------------------------------------
// Formatiert und gibt eine Log-Zeile aus (Implementierung in log.cpp).
// Direkt aufrufen muss man sie normalerweise nicht – die Makros oben reichen.
//
// Das format-Attribut lässt den Compiler prüfen, ob Formatstring und Argumente
// zusammenpassen (z. B. "%d" mit einem int); bei Fehlern warnt er beim Build.
void logWrite(esp_log_level_t level, const char* tag, const char* fmt, ...)
    __attribute__((format(printf, 3, 4)));
