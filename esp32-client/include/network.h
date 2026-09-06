#pragma once

// =============================================================================
// network.h  –  WLAN, DNS und TCP-Kommunikation
//
// Kapselt alles Netzwerk-bezogene. Der ConnectionTask (connection_task.cpp)
// ruft diese Funktionen auf, um die Verbindung aufzubauen und Pakete zu
// senden/empfangen. Der Datenaustausch läuft über den globalen Socket `sock`.
// =============================================================================

#include <cstddef>
#include <cstdint>

// ------------------------- WLAN ---------------------------------------
// Initialisiert WLAN (netif, wifi_init, Event-Handler, Stromsparmodus aus).
void wifiInit();

// Verbindet mit dem angegebenen WLAN und wartet auf die IPv4-Adresse.
// Gibt true zurück, wenn alles geklappt hat.
bool connectWifi(const char ssid[], const char password[]);

// Wartet, bis per DHCP eine IPv4-Adresse vergeben wurde. Muss nach jeder
// WLAN-(Re-)Verbindung aufgerufen werden. Gibt true zurück, wenn eine
// IPv4-Adresse vorhanden ist.
bool waitForIPv4Address();

bool wifiIsConnected();  // true = mit einem Access Point verbunden
int wifiRssi();          // Signalstärke in dBm (-127 = nicht verbunden)

// Debug-Helfer: WLAN-Abbruchgrund als lesbarer Text.
const char* disconnectReason(uint8_t reason);

// WLAN-Abbruch-Merker (siehe Event-Handler): wird bei "endgültigen" Fehlern
// (NO_AP_FOUND, AUTH_FAIL, ...) gesetzt. connectWifi() UND der Reconnect-Pfad
// in connection_task.cpp nutzen ihn, um ihr Warten frühzeitig abzubrechen.
void resetWifiAbort();      // Merker zurücksetzen (vor einem Verbindungsversuch)
bool wifiAbortRequested();  // true = Treiber hat einen endgültigen Fehler gemeldet

// ------------------------- DNS + TCP ----------------------------------
bool resolveDNS();                  // Hostname -> IPv4-Adresse auflösen
bool connectTCP(int retry_counter); // TCP-Socket öffnen und verbinden
void disconnectTCP();               // Socket schließen
bool checkSocket(bool& readable); // blockierend auf eingehende Daten warten

// ------------------------- Senden / Empfangen -------------------------
bool sendAll(const uint8_t* data, size_t len);   // komplette Daten senden
bool recvAll(uint8_t* bytes, size_t bytesToRead); // exakt so viele Bytes lesen
void receivePackage();  // ein Paket vom Server empfangen -> playbackQueue
void sendPackage();     // ein Paket aus sendQueue -> Server schicken
void sendRegister();    // eigene MAC-Adresse registrieren (nach Verbindungsaufbau)
void resendPendingSends(); // nach Reconnect: unbestätigte Sendungen erneut senden

