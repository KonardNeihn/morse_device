#pragma once

// =============================================================================
// tasks.h  –  Deklaration der FreeRTOS-Task-Funktionen
//
// Jeder Task ist eine Funktion mit der Signatur `void(void*)` und läuft als
// eigene Endlosschleife. Die Implementierungen liegen in den gleichnamigen
// .cpp-Dateien. Gestartet werden sie in main.cpp mit xTaskCreatePinnedToCore().
// =============================================================================

void ConnectionTask(void* pvParameters);  // Netzwerk-Zustandsmaschine (CPU 0)
void InputTask(void* pvParameters);       // Taste abtasten -> Morse-Paket (CPU 1)
void PlaybackTask(void* pvParameters);    // Morse-Paket -> Ton + LED (CPU 1)
void CheckerTask(void* pvParameters);     // Drehschalter + Status-LED (CPU 1)
void PrintTask(void* pvParameters);       // Morse-Paket -> Thermodrucker (CPU 1)

