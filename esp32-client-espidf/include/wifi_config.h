#pragma once

// =============================================================================
// wifi_config.h  –  persistente WLAN-Zugangsdaten (NVS)
//
// Speichert SSID/Passwort für das primäre und das Fallback-WLAN im
// nichtflüchtigen Speicher (NVS). So überleben die Daten einen Neustart und
// können über den Hotspot-Modus (Webseite) geändert werden.
// =============================================================================

// Lädt die gespeicherten Werte aus dem NVS in den Arbeitsspeicher.
// Wird einmal beim Start aufgerufen.
void loadWifiConfig();

// Speichert alle vier Werte dauerhaft (NVS) und aktualisiert sofort die
// In-Memory-Werte, damit sie ohne Neustart nutzbar sind.
bool saveWifiConfig(const char* ssid1, const char* pw1,
                    const char* ssid2, const char* pw2);

// Getter auf die aktuell geladenen Werte (leere Strings, wenn nichts gesetzt).
const char* getSsid1();
const char* getPassword1();
const char* getSsid2();
const char* getPassword2();

// true, wenn mindestens das primäre WLAN konfiguriert ist.
bool hasWifiConfig();
