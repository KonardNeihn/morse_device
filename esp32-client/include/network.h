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

// Verbindet mit dem angegebenen WLAN und wartet auf die IPv6-Adresse.
// Gibt true zurück, wenn alles geklappt hat.
bool connectWifi(const char ssid[], const char password[]);

// Wartet, bis eine globale IPv6-Adresse (und damit eine Route zum Server)
// vorhanden ist. Muss nach jeder WLAN-(Re-)Verbindung aufgerufen werden,
// weil die globale Adresse per Router Advertisement erst nach einiger Zeit
// vergeben wird. Gibt true zurück, wenn IPv6 einsatzbereit ist.
bool waitForIPv6Address();

bool wifiIsConnected();  // true = mit einem Access Point verbunden
int wifiRssi();          // Signalstärke in dBm (-127 = nicht verbunden)

// Debug-Helfer: WLAN-Abbruchgrund als lesbarer Text.
const char* disconnectReason(uint8_t reason);

// ------------------------- DNS + TCP ----------------------------------
bool resolveDNS();                  // Hostname -> IPv6-Adresse auflösen
bool connectTCP(int retry_counter); // TCP-Socket öffnen und verbinden
void disconnectTCP();               // Socket schließen
bool checkSocket(bool& readable); // blockierend auf eingehende Daten warten

// ------------------------- Senden / Empfangen -------------------------
bool sendAll(const uint8_t* data, size_t len);   // komplette Daten senden
bool recvAll(uint8_t* bytes, size_t bytesToRead); // exakt so viele Bytes lesen
void receivePackage();  // ein Paket vom Server empfangen -> playbackQueue
void sendPackage();     // ein Paket aus sendQueue -> Server schicken

