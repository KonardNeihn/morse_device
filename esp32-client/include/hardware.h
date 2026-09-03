#pragma once

// Status-Anzeige über die LED (Blinkmuster je Verbindungszustand).
void showSearchingWiFi();
void showWaitForIP6();
void showResolvingDNS();
void showConnectTCP();
void showBadSignal();

// Ton (nur wenn der Drehschalter "ohne Ton" nicht aktiv ist).
void playTone();

// Drehschalter einlesen und in die globalen Modus-Flags schreiben.
void checkPins();

// MOSFET kurz ausschalten, damit der ESP32 bei Rückstrom einfach ausgeht.
void testMosfet();
