#pragma once

// FreeRTOS-Task-Funktionen (Signatur nach TaskFunction_t).
void ConnectionTask(void* pvParameters);
void InputTask(void* pvParameters);
void PlaybackTask(void* pvParameters);
void CheckerTask(void* pvParameters);
void PrintTask(void* pvParameters);
