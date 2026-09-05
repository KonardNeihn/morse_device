// =============================================================================
// printer.cpp  –  Thermodrucker über UART2 (ESC/POS-Protokoll)
//
// Der Drucker wird mit rohen ESC/POS-Kommandos angesteuert:
//   ESC @   -> Drucker zurücksetzen
//   ESC 7   -> Heizparameter einstellen
//   ESC *   -> Grafikmodus starten
// Jede "Spalte" des Druckbilds besteht aus 8 vertikalen Punkten (1 Byte).
// =============================================================================

#include "printer.h"

#include <driver/uart.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "config.h"

static const char* TAG = "printer";
static const uart_port_t PRINTER_UART = UART_NUM_2;  // UART2 für den Drucker

static void writeByte(uint8_t byte) {
  uart_write_bytes(PRINTER_UART, &byte, 1);  // ein einzelnes Byte zum Drucker
}

void printerInit() {
  // UART-Konfiguration: 9600 Baud, 8 Datenbits, keine Parität, 1 Stoppbit.
  uart_config_t cfg = {};
  cfg.baud_rate = 9600;                   // Übertragungsgeschwindigkeit
  cfg.data_bits = UART_DATA_8_BITS;       // 8 Datenbits pro Byte
  cfg.parity = UART_PARITY_DISABLE;       // kein Paritätsbit
  cfg.stop_bits = UART_STOP_BITS_1;       // 1 Stoppbit
  cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;  // keine Flusskontrolle
  cfg.source_clk = UART_SCLK_DEFAULT;     // Standard-Taktquelle

  // UART-Treiber installieren (2048 Bytes Sende-Puffer).
  ESP_ERROR_CHECK(uart_driver_install(PRINTER_UART, 2048, 0, 0, NULL, 0));
  // Baudrate & Co. auf den UART anwenden.
  ESP_ERROR_CHECK(uart_param_config(PRINTER_UART, &cfg));
  // Dem UART die GPIO-Pins zuweisen (TX=17, RX=16).
  ESP_ERROR_CHECK(uart_set_pin(PRINTER_UART, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

  ESP_LOGI(TAG, "UART2 initialisiert (TX=%d RX=%d)", TX_PIN, RX_PIN);
}

void makePrinterReady() {
  // ESC @ : Drucker auf Werkszustand zurücksetzen.
  writeByte(27);   // ESC
  writeByte('@');  // @
  vTaskDelay(1);

  // ESC 7 n1 n2 n3 : Heizparameter konfigurieren.
  writeByte(27);   // ESC
  writeByte(55);   // 7
  writeByte(1);    // n1: Punkte, die gleichzeitig heizen dürfen
  writeByte(255);  // n2: Heizzeit (hoch = langsamer, aber dunkler)
  writeByte(255);  // n3: Pause zwischen Heizungen (hoch = weniger Strom)
  vTaskDelay(1);
}

void print(bool top_line[384], bool bottom_line[384]) {
  // ESC * 1 nL nH : Grafikmodus "8-dot double density" starten.
  writeByte(27);
  writeByte('*');
  writeByte((uint8_t)1);    // Mode 1 = 8 Punkte vertikal, doppelte Dichte
  writeByte((uint8_t)128);  // nL = 128 Bits
  writeByte((uint8_t)1);    // nH = 1 -> Zeilenlänge = nL + nH*256 = 384
  vTaskDelay(1);

  for (int i = 0; i < 384; i++) {
    uint8_t column = 0b00010000;  // feste Trennlinie zwischen den zwei Zeilen
    if (top_line[i])
      column = column | 0b11000000;  // Punkte für die obere Zeile
    if (bottom_line[i])
      column = column | 0b00000110;  // Punkte für die untere Zeile

    writeByte(column);  // die Spalte zum Drucker senden

    if (i % 16 == 0)  // alle 16 Spalten kurz "durchatmen" (gibt dem Watchdog Luft)
      vTaskDelay(1);
  }
  vTaskDelay(pdMS_TO_TICKS(50));
}

void printerWriteByte(uint8_t byte) {
  writeByte(byte);
}

void printerFlush() {
  uart_flush(PRINTER_UART);  // warten, bis der UART-Puffer leer ist
}

