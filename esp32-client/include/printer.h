#pragma once

// Thermodrucker (ESC/POS) auf UART2.
// makePrinterReady() konfiguriert den Drucker (Reset + Heizparameter),
// print() gibt eine Zeile aus zwei übereinanderliegenden Punktreihen aus.

void makePrinterReady();
void print(bool top_line[384], bool bottom_line[384]);
