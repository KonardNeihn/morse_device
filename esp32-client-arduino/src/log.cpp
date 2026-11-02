// =============================================================================
// log.cpp  –  Implementierung der Log-Ausgabe
//
// Schreibt farbige, aufgeräumte Log-Zeilen über Serial. Jede Zeile hat den
// Aufbau (genau wie in der ESP-IDF-Version):
//
//     <Farbe><Level-Buchstabe> (Tage:HH:MM:SS.mmm) <Tag>: <Meldung>
//
// Beispiel:
//     I (00:00:01:02.345) network    : Verbinde mit morse-server.de
//
// -----------------------------------------------------------------------------
// Unterschied zur ESP-IDF-Version:
//
// Dort wurde esp_log_set_vprintf() benutzt, um die fertig formatierten
// ESP_LOGx-Ausgaben umzufärben. Das erforderte ein fragiles Nachparsen des
// ESP-Log-Formats (strstr("%s:") etc.). Hier bauen wir die Zeile dagegen
// selbst zusammen und geben sie in einem Rutsch aus. Das ist:
//
//   - robuster     (wir besitzen das Format, nichts muss interpretiert werden)
//   - Arduino-nah  (Ausgabe über Serial, wie in Sketches üblich)
//
// Außerdem entfällt die Abhängigkeit von esp_private/log_lock.h und dem
// esp_log_set_vprintf-Hook komplett.
// =============================================================================

#include "log.h"

#include <Arduino.h>  // Serial, millis()
#include <stdarg.h>   // va_list, va_start, va_end
#include <stdio.h>    // snprintf, vsnprintf

// ------------------------- Puffer & Helfer --------------------------------
// Puffergröße für eine Log-Zeile. 192 Byte reichen für übliche Meldungen;
// bei Bedarf einfach vergrößern.
static constexpr size_t LOG_BUF_SIZE = 192;

// Kappt eine Länge auf den gültigen Pufferbereich (0 .. LOG_BUF_SIZE-1).
// snprintf/vsnprintf liefern die Länge zurück, die geschrieben worden WÄRE –
// die kann den Puffer übersteigen. Ohne Kappen würde man sonst über das
// Pufferende hinausschreiben.
static int clampLen(int n) {
    if (n < 0) return 0;
    if (n >= (int)LOG_BUF_SIZE) return LOG_BUF_SIZE - 1;
    return n;
}

// ------------------------- Farbe je Level --------------------------------
// Liefert die ANSI-Escape-Sequenz für die Vordergrundfarbe des Levels.
// "\033[1;3Xm": 1 = kräftig, 3X = Farbe (1 rot, 3 gelb, 2 grün, 6 cyan,
// 5 magenta), m = Ende der Sequenz.
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

// ------------------------- Level -> Kennbuchstabe ------------------------
// Übersetzt einen esp_log_level_t-Wert in den einzelnen Buchstaben, der in
// der Zeile vor der Zeitangabe steht (E/W/I/D/V). In der ESP-IDF-Version
// steckte dieser Buchstabe bereits im Formatstring der ESP_LOGx-Makros –
// da wir die Zeile jetzt selbst bauen, brauchen wir ihn als eigene Funktion.
static char levelChar(esp_log_level_t level) {
    switch (level) {
        case ESP_LOG_ERROR:   return 'E';  // Fehler
        case ESP_LOG_WARN:    return 'W';  // Warnung
        case ESP_LOG_INFO:    return 'I';  // Info
        case ESP_LOG_DEBUG:   return 'D';  // Debug
        case ESP_LOG_VERBOSE: return 'V';  // Verbose
        default:              return '?';  // unbekanntes Level
    }
}

// ------------------------- zentrale Ausgabe ------------------------------
void logWrite(esp_log_level_t level, const char* tag, const char* fmt, ...) {
    char buf[LOG_BUF_SIZE];

    // Laufzeit seit dem Boot (millis()) in Tage:Stunden:Minuten:Sekunden.
    // Millisekunden zerlegen.
    unsigned long milli = millis();
    unsigned long sec   = (milli / 1000) % 60;      // Sekunden (0..59)
    unsigned long min   = (milli / 60000) % 60;     // Minuten  (0..59)
    unsigned long hrs   = (milli / 3600000) % 24;   // Stunden  (0..23)
    unsigned long day   = milli / 86400000;         // ganze Tage
    unsigned long ms    = milli % 1000;             // Millisekunden (0..999)

    // Kopf der Zeile: <Farbe><Buchstabe> (Tage:HH:MM:SS.mmm) <Tag>:
    //   %-12s  = Tag linksbündig auf 12 Zeichen (damit Meldungen fluchten)
    char lc = levelChar(level);
    int n = snprintf(buf, sizeof(buf),
                     "%s%c\033[0m (%02lu:%02lu:%02lu:%02lu.%03lu) %-12s: ",
                     levelColor(lc), lc, day, hrs, min, sec, ms, tag);
    n = clampLen(n);

    // Eigentliche Meldung mit den variablen Argumenten anhängen.
    va_list args;
    va_start(args, fmt);
    // clampLen() begrenzt n auf LOG_BUF_SIZE-1, damit buf+n nie den Puffer
    // verlässt – ohne diesen Schutz wäre die Zeiger-Arithmetik undefiniert.
    n = clampLen(n + vsnprintf(buf + n, sizeof(buf) - n, fmt, args));
    va_end(args);

    // Farbe zurücksetzen + Zeilenumbruch anhängen.
    n = clampLen(n + snprintf(buf + n, sizeof(buf) - n, "\033[0m\n"));

    // Die ganze Zeile in EINEM write() ausgeben. Das verhindert, dass sich
    // Ausgaben mehrerer Tasks mitten in einer Zeile verschachteln.
    Serial.write((const uint8_t*)buf, n);
}