#pragma once

// =============================================================================
// network.h  –  WLAN, DNS und TCP-Kommunikation
//
// Kapselt alles Netzwerk-bezogene. Der ConnectionTask (connection_task.cpp)
// ruft diese Funktionen auf, um die Verbindung aufzubauen und Pakete zu
// senden/empfangen. Der Datenaustausch läuft über ein internes `WiFiClient`-
// Objekt (siehe network.cpp) – kein roher lwIP-Socket mehr.
// =============================================================================

#include <cstdint>   // uint8_t, uint64_t

// ------------------------- WLAN -------------------------------------------
void wifiInit();                                // initialisieren (Mode, Events, MAC)
bool wifiIsConnected();                         // true = mit einem AP verbunden
int wifiRssi();                                 // Signalstärke in dBm (-127 = nicht verbunden)
const char* disconnectReason(uint8_t reason);   // Abbruchgrund als lesbarer Text (Debug)
void configureWifiCredentials();
void connectWifi();

// ------------------------- TCP ---------------------------------------------
bool connectTCP();                              // Verbindung zum Server aufbauen
void disconnectTCP();                           // Verbindung schließen
bool tcpIsConnected();                          // true = Socket offen
bool tcpAvailableMessage();                     // true = mindestens ein Byte im Empfangspuffer

// ------------------------- Senden / Empfangen ------------------------------
void sendPackage();                             // ein Paket aus sendQueue -> Server
void receivePackage();                          // ein Paket vom Server -> playbackQueue
void sendRegister();                            // eigene MAC-Adresse registrieren
void resendPendingSends();                      // unbestätigte Sendungen nach Reconnect erneut senden
void queueDeliveryAck(uint64_t server_msg_id);  // Zustell-ACK nach dem Druck vormerken
