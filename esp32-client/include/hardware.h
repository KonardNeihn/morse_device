#pragma once

// =============================================================================
// hardware.h  –  Zugriff auf die angeschlossene Hardware
//
// Bündelt alles, was direkt mit den Pins zu tun hat:
// GPIOs (Taste, LED, MOSFET, Drehschalter) und LEDC (Lautsprecher-PWM).
// =============================================================================

// Initialisiert GPIOs (Taste, LED, MOSFET, Drehschalter) und LEDC (Lautsprecher).
void hardwareInit();

// Status-LED: `blinks`-mal kurz aufleuchten lassen. Die Anzahl entspricht dem
// Verbindungszustand (1x = WLAN suchen, 2x = IP6 warten, 3x = DNS, 4x = TCP, 5x = bad Signal).
void showStatus(int blinks);

// Ton an/aus (nur wenn der Drehschalter "ohne Ton" NICHT aktiv ist).
void playTone();
void stopTone();

// Drehschalter einlesen und die globalen Modus-Flags (NO_SOUND_MODE etc.) setzen.
void checkPins();

// MOSFET kurz ausschalten, damit der ESP32 bei Rückstrom einfach ausgeht
// (Sicherheits-Mechanik der Stromversorgung).
void testMosfet();

