// =============================================================================
// storage.cpp  –  persistente WLAN-Zugangsdaten (Preferences/NVS)
//
// Lädt und speichert SSID/Passwort für das primäre und das Fallback-WLAN.
// Als Backend dient die Arduino-Bibliothek "Preferences", die selbst nur eine
// bequeme Schicht über ESP-IDF-NVS ist. Damit landen die Daten in denselben
// Flash-Sektoren wie bei nvs_open()/nvs_set_str() in der ESP-IDF-Version.
//
// Wichtig gegenüber der ESP-IDF-Version:
//   - Kein nvs_flash_init() nötig (macht der Arduino-Core beim Boot selbst).
//   - Kein expliziter commit() – end() schreibt automatisch.
//   - Kein nvs_handle_t – Preferences kapselt das mit begin()/end().
// =============================================================================

#include "storage.h"
#include "log.h"

#include <Preferences.h>
#include <string.h>   // strlcpy

static const char* TAG = "storage";

// Preferences-"Namespace": gruppiert die vier Schlüssel. Entspricht dem
// NVS-Namespace "wifi" der ESP-IDF-Referenz (bei einem Wechsel des Backends
// bleiben die Daten also lesbar).
static constexpr const char* PREF_NS = "wifi";

// Schlüssel für die vier gespeicherten Felder.
static constexpr const char* KEY_SSID1 = "ssid1";
static constexpr const char* KEY_PW1   = "pw1";
static constexpr const char* KEY_SSID2 = "ssid2";
static constexpr const char* KEY_PW2   = "pw2";

// Maximale Längen (passend zu wifi_config_t: SSID max. 32, Passwort max. 64).
static constexpr size_t SSID_MAX = 32;
static constexpr size_t PASS_MAX = 64;

// Aktuell geladene Werte im RAM (auch ohne Neustart sofort nutzbar und von
// den Gettern unten zurückgegeben). `+1` für das abschließende '\0'.
static char ssid1[SSID_MAX + 1] = { 0 };
static char pw1[PASS_MAX + 1]   = { 0 };
static char ssid2[SSID_MAX + 1] = { 0 };
static char pw2[PASS_MAX + 1]   = { 0 };

void loadWifiConfig() {
    // read-only öffnen: zum Laden reicht Lesezugriff.
    Preferences prefs;
    if (!prefs.begin(PREF_NS, true)) {
        LOGI(TAG, "keine gespeicherte WLAN-Konfiguration gefunden");
        return;
    }

    // getString(key, buf, maxLen) schreibt höchstens maxLen-1 Zeichen + '\0'
    // und gibt die tatsächliche Länge zurück. Existiert der Schlüssel nicht,
    // bleibt buf[0] == '\0' (leerer String) – kein zusätzlicher Check nötig.
    prefs.getString(KEY_SSID1, ssid1, sizeof(ssid1));
    prefs.getString(KEY_PW1,   pw1,   sizeof(pw1));
    prefs.getString(KEY_SSID2, ssid2, sizeof(ssid2));
    prefs.getString(KEY_PW2,   pw2,   sizeof(pw2));

    prefs.end();  // nur lesen -> kein commit nötig

    LOGI(TAG, "WLAN-Konfiguration geladen");
    // Nur auf Debug-Level sichtbar (LOGD ist bei LOG_LEVEL=INFO wegkompiliert).
    LOGD(TAG, "ssid1: %s \t pw1: %s", ssid1, pw1);
    LOGD(TAG, "ssid2: %s \t pw2: %s", ssid2, pw2);
}

bool saveWifiConfig(const char* s1, const char* p1,
                    const char* s2, const char* p2) {
    // 1) In-Memory sofort aktualisieren -> ohne Neustart nutzbar.
    //    nullptr/leer wird als "leer" übernommen (strlcpy ist nullptr-sicher
    //    nicht, deshalb der explizite Zeiger-Check).
    if (s1) strlcpy(ssid1, s1, sizeof(ssid1)); else ssid1[0] = '\0';
    if (p1) strlcpy(pw1,   p1, sizeof(pw1));   else pw1[0]   = '\0';
    if (s2) strlcpy(ssid2, s2, sizeof(ssid2)); else ssid2[0] = '\0';
    if (p2) strlcpy(pw2,   p2, sizeof(pw2));   else pw2[0]   = '\0';

    // 2) read-write öffnen, um zu schreiben.
    Preferences prefs;
    if (!prefs.begin(PREF_NS, false)) {
        LOGE(TAG, "Preferences in read-write öffnen fehlgeschlagen");
        return false;
    }

    // Gefüllte Werte speichern, leere Werte aus dem Flash entfernen
    // (so taucht kein leerer String auf, der "konfiguriert" vorgaukelt).
    auto write = [&](const char* key, const char* val) {
        if (val && val[0] != '\0')
            prefs.putString(key, val);
        else
            prefs.remove(key);
    };
    write(KEY_SSID1, ssid1);
    write(KEY_PW1,   pw1);
    write(KEY_SSID2, ssid2);
    write(KEY_PW2,   pw2);

    prefs.end();  // schreibt alle Änderungen automatisch in den Flash

    LOGI(TAG, "WLAN-Konfiguration gespeichert");
    return true;
}

const char* getSsid1()     { return ssid1; }
const char* getPassword1() { return pw1; }
const char* getSsid2()     { return ssid2; }
const char* getPassword2() { return pw2; }

bool hasWifiConfig() { return ssid1[0] != '\0'; }