// =============================================================================
// main.cpp  –  Einstiegspunkt (app_main)
//
// app_main() ist unter ESP-IDF das Äquivalent zu main(). Hier wird in dieser
// Reihenfolge alles hochgefahren:
//
//   1. NVS      – nichtflüchtiger Flash-Speicher (wird von WLAN/TCP gebraucht)
//   2. netif    – Netzwerk-Schnittstellen + Event-Loop (WLAN-Ereignisse)
//   3. Treiber  – WLAN, Hardware (GPIO/LEDC), Drucker (UART)
//   4. Queues   – Nachrichtenkanäle zwischen den Tasks
//   5. Tasks    – je eine eigene Endlosschleife, auf die CPU-Kerne verteilt
//
// Danach läuft alles nebenläufig über FreeRTOS-Tasks weiter.
// =============================================================================

#include <esp_event.h>
#include <esp_idf_version.h>
#include <esp_log.h>
#include <esp_netif.h>
#include <nvs_flash.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "esp_private/log_lock.h"

#include "app_state.h"
#include "config.h"
#include "hardware.h"
#include "network.h"
#include "package.h"
#include "printer.h"
#include "tasks.h"

static const char* TAG = "main";

// Log-Ausgaben (gibt es überall im Projekt):
//   ESP_LOGI(TAG, ...)  -> Info-Meldung   (normale Ausgabe)
//   ESP_LOGW(TAG, ...)  -> Warnung        (ungewöhnlich, aber kein Fehler)
//   ESP_LOGE(TAG, ...)  -> Fehler         (etwas ist schiefgelaufen)
// TAG ist ein kurzer Name, der in der Konsole vor jeder Zeile steht
// (z. B. "main", "network", "hardware" ...), damit man weiß, woher die Meldung kommt.

// =============================================================================
// Farbige, aufgeräumte Log-Ausgabe
// =============================================================================
// ESP-IDF (Log-Format v1) ruft diese Funktion mit verschiedenen Format-
// Varianten auf. Nur das ESP_LOGx-Makro hat ein sicheres, bekanntes Format:
//
//   "<Level> (%u) %s: <Meldung>\n"   (Level wörtlich, ts als %u, TAG als %s)
//
// Alle anderen Varianten (WLAN-Treiber "%c (%d) ...", rohe esp_log_write-
// Aufrufe) sind intern unterschiedlich aufgebaut -> die NICHT interpretieren,
// sondern nur einfärben und unverändert durchreichen. Ein falsches Auslesen
// der Argumente würde hier zum Absturz führen.
static const char* levelColor(char level) {
  switch (level) {
    case 'E': return "\033[1;31m";  // rot
    case 'W': return "\033[1;33m";  // gelb
    case 'I': return "\033[1;32m";  // grün
    case 'D': return "\033[1;36m";  // cyan
    case 'V': return "\033[1;35m";  // magenta
    default:  return "\033[0m";     // unbekannt -> keine Farbe
  }
}

static int prettyLogVprintf(const char* fmt, va_list args) {
  // Nur das ESP_LOGx-Makro erkennen (Level-Buchstabe + " (%").
  const bool isMacro =
      (fmt[0] == 'E' || fmt[0] == 'W' || fmt[0] == 'I' ||
       fmt[0] == 'D' || fmt[0] == 'V') &&
      fmt[1] == ' ' && fmt[2] == '(' && fmt[3] == '%';

  esp_log_impl_lock();  // gegen Zerreißen durch andere Tasks (Re-Entranz)
  int n;

  if (isMacro) {
    // ---------- ESP_LOGx-Makro: vollständig neu formatieren ----------
    char level = fmt[0];
    uint32_t ms = va_arg(args, uint32_t);
    const char* tag = va_arg(args, const char*);

    unsigned h     = ms / 3600000u;
    unsigned m     = (ms / 60000u) % 60u;
    unsigned s     = (ms / 1000u) % 60u;
    unsigned milli = ms % 1000u;

    // Meldung: alles nach dem "%s:"-Platzhalter des TAG (evtl. + Leerzeichen).
    const char* msg = strstr(fmt, "%s:");
    if (msg) {
      msg += 3;
      if (msg[0] == ' ')
        msg++;
    } else {
      msg = fmt;
    }

    n = printf("%s%c\033[0m (%02u:%02u:%02u.%03u) %-12s: ",
               levelColor(level), level, h, m, s, milli, tag);
    n += vprintf(msg, args);
  } else {
    // ---------- alles andere: nur einfärben, unverändert ausgeben ----------
    // Level aus fmt[0] oder (bei "%c ...") aus dem ersten Argument ermitteln,
    // OHNE die Argumente zu verbrauchen.
    char level = 0;
    if (fmt[0] == 'E' || fmt[0] == 'W' || fmt[0] == 'I' ||
        fmt[0] == 'D' || fmt[0] == 'V') {
      level = fmt[0];
    } else if (fmt[0] == '%' && fmt[1] == 'c') {
      va_list copy;
      va_copy(copy, args);
      level = (char)va_arg(copy, int);
      va_end(copy);
    }

    if (level == 0) {
      n = vprintf(fmt, args);  // Level unbekannt -> ohne Farbe
    } else {
      n = printf("%s", levelColor(level));
      n += vprintf(fmt, args);
      n += printf("\033[0m");
    }
  }

  esp_log_impl_unlock();
  return n;
}

// Hängt unsere hübsche Log-Ausgabe ein (ersetzt die Standard-Ausgabe).
static void logInit() {
  esp_log_set_vprintf(&prettyLogVprintf);
}

extern "C" void app_main(void) {
  // Hinweis: ESP_ERROR_CHECK(x) prüft den Rückgabewert von x. Ist er ein
  // Fehler, wird das Programm mit einer Fehlermeldung abgebrochen.
  // (Wird in diesem Projekt überall zum Abfangen von ESP-IDF-Fehlern genutzt.)

  // Farbige, aufgeräumte Log-Ausgabe aktivieren (vor allen anderen Meldungen).
  logInit();

  // NVS (Non-Volatile Storage, "Flash-Speicher") initialisieren.
  // WLAN/TCP speichern dort intern ihre Einstellungen.
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    // Speicher voll oder alte Version -> einmal löschen und neu versuchen.
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  // Netzwerk-Schnittstellen und die Event-Loop (für WLAN-Ereignisse) anlegen.
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());

  // Treiber initialisieren (Reihenfolge: erst WLAN, dann GPIO/LEDC, dann UART).
  wifiInit();
  hardwareInit();
  printerInit();

  // Version der ESP-IDF ausgeben (nur zu Info-Zwecken).
  ESP_LOGI(TAG, "ESP-IDF %s", esp_get_idf_version());

  // Die drei Queues sind die Kommunikationskanäle zwischen den Tasks.
  // Durchgeschoben werden Zeiger auf Package-Objekte (der Inhalt liegt im Heap).
  sendQueue     = xQueueCreate(QUEUE_SIZE, sizeof(Package*));  // Eingabe -> Netz
  playbackQueue = xQueueCreate(QUEUE_SIZE, sizeof(Package*));  // Netz -> Wiedergabe
  printQueue    = xQueueCreate(QUEUE_SIZE, sizeof(Package*));  // Wiedergabe -> Druck

  // Alle Tasks in einer Tabelle beschreiben: { Funktion, Name, Priorität, CPU-Kern }.
  // Priorität: höher = wichtiger (bekommt bei Konkurrenz zuerst CPU-Zeit).
  struct TaskDef {
    TaskFunction_t func;   // die Task-Funktion (Signatur: void(void*))
    const char* name;      // Name (nur für Debug-Ausgaben)
    int priority;          // Priorität (0..N, höher = wichtiger)
    int core;              // CPU-Kern (0 oder 1)
  };
  constexpr int TASK_STACK_SIZE = 4096;  // Stack pro Task in Bytes
  const TaskDef tasks[] = {
    { ConnectionTask, "Check WiFi TCP", 1, 0 },  // Netzwerk-Zustandsmaschine
    { InputTask,      "Input Task",     1, 1 },  // Morse-Taste abtasten
    { PlaybackTask,   "Output Task",    1, 1 },  // Ton + LED
    { CheckerTask,    "Checker Task",   2, 1 },  // Drehschalter + Status-LED
    { PrintTask,      "Print Task",     2, 1 },  // Thermodrucker
  };

  // Jeden Task starten und fest an seinen CPU-Kern binden ("pinned").
  for (const TaskDef& t : tasks)
    xTaskCreatePinnedToCore(t.func, t.name, TASK_STACK_SIZE, NULL, t.priority, NULL, t.core);
}


