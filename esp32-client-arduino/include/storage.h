#pragma once

// =============================================================================
// storage.h  –  persistente WLAN-Zugangsdaten
//
// Speichert SSID/Passwort für das primäre und das Fallback-WLAN im
// nichtflüchtigen Flash-Speicher. Genutzt wird die Arduino-Bibliothek
// "Preferences", die intern auf ESP-IDF-NVS aufsetzt. So überleben die Daten
// einen Neustart und können über den Hotspot-Modus (Webseite) geändert werden.
// =============================================================================

// Lädt die gespeicherten Werte aus dem Flash in den Arbeitsspeicher.
// Wird einmal beim Start aufgerufen; wenn nichts gespeichert ist, bleiben
// die Puffer leer ("").
void loadWifiConfig();

// Speichert alle vier Werte dauerhaft und aktualisiert sofort die
// In-Memory-Werte, damit sie ohne Neustart nutzbar sind.
// Wird nullptr oder ein leerer String übergeben, wird der jeweilige Wert
// gelöscht (statt einen leeren String zu speichern).
bool saveWifiConfig(const char* ssid1, const char* pw1,
                    const char* ssid2, const char* pw2);

// Getter auf die aktuell geladenen Werte (leere Strings, wenn nichts gesetzt).
const char* getSsid1();
const char* getPassword1();
const char* getSsid2();
const char* getPassword2();

// true, wenn mindestens das primäre WLAN (SSID1) konfiguriert ist.
bool hasWifiConfig();