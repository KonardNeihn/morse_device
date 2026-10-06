// =============================================================================
// hardware.cpp  –  Initialisierung und Ansteuerung der Hardware
//
// Konfiguriert alle GPIOs und den LEDC-Kanal (PWM) für den Lautsprecher.
// Außerdem liegen hier die kleinen Helfer für LED-Blinkmuster, Ton und das
// Einlesen der Drehschalter.
// =============================================================================

#include "hardware.h"

#include <driver/gpio.h>
#include <driver/ledc.h>
#include <esp_log.h>
#include "esp_adc/adc_oneshot.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "app_state.h"
#include "config.h"

static const char* TAG = "hardware";

// ADC-Handle für den Drehschalter (wird in hardwareInit() angelegt).
static adc_oneshot_unit_handle_t adc_handle = nullptr;

// LEDC-Konfiguration für den Lautsprecher (PWM).
// LEDC erzeugt ein Rechtecksignal mit einstellbarer Frequenz (Tonhöhe) und
// Tastverhältnis ("duty" = Lautstärke).
#define LEDC_MODE    LEDC_LOW_SPEED_MODE   // langsame Betriebsart reicht für Ton
#define LEDC_TIMER   LEDC_TIMER_0          // Timer 0
#define LEDC_CHANNEL LEDC_CHANNEL_0        // Kanal 0
#define LEDC_RES     LEDC_TIMER_8_BIT      // 8-Bit-Auflösung (Werte 0..255)
#define LEDC_DUTY_ON 128                   // 50 % Tastverhältnis = halbe Lautstärke

void hardwareInit() {
  // --- Ausgänge: LED und MOSFET. ---
  gpio_config_t out = {};                               // Konfigurations-Struktur
  out.pin_bit_mask = (1ULL << LED) | (1ULL << MOSFET);  // diese Pins auswählen
  out.mode = GPIO_MODE_OUTPUT;                          // als Ausgang nutzen
  out.pull_up_en = GPIO_PULLUP_DISABLE;                 // kein interner Pull-up
  out.pull_down_en = GPIO_PULLDOWN_DISABLE;             // kein interner Pull-down
  out.intr_type = GPIO_INTR_DISABLE;                    // keine Interrupts
  ESP_ERROR_CHECK(gpio_config(&out));  // Konfiguration auf die Pins anwenden

  // MOSFET einschalten: Der ESP32 hält sich damit selbst mit Strom
  // (Selbsthalte-Schaltung, damit er nach dem Einschalten weiterläuft).
  gpio_set_level((gpio_num_t)MOSFET, 1);

  // --- Eingang: Morse-Taste mit internem Pull-up (LOW = gedrückt). ---
  gpio_config_t button = {};
  button.pin_bit_mask = (1ULL << BUTTON);
  button.mode = GPIO_MODE_INPUT;
  button.pull_up_en = GPIO_PULLUP_ENABLE;
  button.pull_down_en = GPIO_PULLDOWN_DISABLE;
  button.intr_type = GPIO_INTR_DISABLE;
  ESP_ERROR_CHECK(gpio_config(&button));

  // --- Drehschalter: 6 Positionen über EINEN ADC-Kanal (Spannungsteiler). ---
  // Der One-Shot-Driver konfiguriert den GPIO intern als Analog-Eingang.
  adc_oneshot_unit_init_cfg_t adc_unit = {};
  adc_unit.unit_id = ADC_UNIT_1;          // der C5 hat nur ADC1
  ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc_unit, &adc_handle));

  adc_oneshot_chan_cfg_t adc_chan = {};
  adc_chan.atten = ROTARY_ATTEN;          // Messbereich ~0..3,1 V (siehe config.h)
  adc_chan.bitwidth = ADC_BITWIDTH_12;    // 12 Bit Auflösung (0..4095)
  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ROTARY_ADC_CHANNEL, &adc_chan));

  // --- LEDC (PWM) für den Lautsprecher. ---
  // LEDC ist der PWM-Controller des ESP32 (erzeugt das Ton-Signal).
  ledc_timer_config_t timer = {};
  timer.speed_mode = LEDC_MODE;          // Betriebsart (langsam reicht für Ton)
  timer.duty_resolution = LEDC_RES;      // Auflösung des Tastverhältnisses (8 Bit)
  timer.timer_num = LEDC_TIMER;          // welcher Timer
  timer.freq_hz = SOUND_FREQ;            // Frequenz = Tonhöhe (siehe config.h)
  timer.clk_cfg = LEDC_AUTO_CLK;         // Taktquelle automatisch wählen
  ESP_ERROR_CHECK(ledc_timer_config(&timer));  // Timer einrichten

  ledc_channel_config_t channel = {};
  channel.gpio_num = SPEAKER;      // PWM-Signal an diesen Pin ausgeben
  channel.speed_mode = LEDC_MODE;
  channel.channel = LEDC_CHANNEL;  // welcher Kanal
  channel.timer_sel = LEDC_TIMER;  // Kanal mit obigem Timer verbinden
  channel.duty = 0;                // Start: Tastverhältnis 0 = Ton aus
  channel.hpoint = 0;
  ESP_ERROR_CHECK(ledc_channel_config(&channel));  // Kanal einrichten

  // WICHTIG: ledc_set_duty_and_update() ist die thread-sichere Variante und
  // nutzt intern den Fade-Dienst zur Glitch-Vermeidung. Ohne diese einmalige
  // Installation schlägt jeder Duty-Wechsel fehl ("Fade service not
  // installed") -> es kommt kein Ton.
  ESP_ERROR_CHECK(ledc_fade_func_install(0));

  ESP_LOGI(TAG, "GPIO + LEDC initialisiert");
}

// Ein einzelner Blitz: LED an, warten, LED aus, warten.
static void blink(int delay_ms) {
  gpio_set_level((gpio_num_t)LED, 1);
  vTaskDelay(pdMS_TO_TICKS(delay_ms));
  gpio_set_level((gpio_num_t)LED, 0);
  vTaskDelay(pdMS_TO_TICKS(delay_ms));
}

// Status-LED: `blinks`-mal kurz aufleuchten lassen. Die Anzahl entspricht dem
// Verbindungszustand (1x = WLAN suchen, 2x = IP6 warten, 3x = DNS, 4x = TCP).
void showStatus(int blinks) {
  for (int i = 0; i < blinks; i++)
    blink(200);
}

void playTone() {
  // Ton nur abspielen, wenn der Drehschalter NICHT auf "ohne Ton" steht.
  if (!NO_SOUND_MODE) {
    // Tastverhältnis auf 50 % setzen -> PWM-Signal läuft -> Ton hörbar.
    ledc_set_duty_and_update(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_ON, 0);
  }
}

void stopTone() {
  // Tastverhältnis auf 0 -> PWM-Signal aus -> Lautsprecher still.
  ledc_set_duty_and_update(LEDC_MODE, LEDC_CHANNEL, 0, 0);
}

void checkPins() {
  // Drehschalter über den ADC einlesen (Spannungsteiler -> 6 Positionen).
  int raw = 0;
  if (adc_oneshot_read(adc_handle, ROTARY_ADC_CHANNEL, &raw) != ESP_OK) {
    ESP_LOGW(TAG, "ADC-Lesen fehlgeschlagen");
    return;  // alte Modus-Flags unverändert lassen
  }

  // Position anhand der Schwellwerte bestimmen (siehe config.h).
  int pos;
  if      (raw < ROTARY_TH_1) pos = 0;  // NORMAL (alle Flags aus)
  else if (raw < ROTARY_TH_2) pos = 1;  // NO_SOUND
  else if (raw < ROTARY_TH_3) pos = 2;  // NO_PRINTER
  else if (raw < ROTARY_TH_4) pos = 3;  // SELF_CHECK
  else if (raw < ROTARY_TH_5) pos = 4;  // SERVER_CHECK
  else                        pos = 5;  // HOTSPOT

  // Genau EIN Modus aktiv; NORMAL = alle Flags false.
  NO_SOUND_MODE     = (pos == 1);
  NO_PRINTER_MODE   = (pos == 2);
  SELF_CHECK_MODE   = (pos == 3);
  SERVER_CHECK_MODE = (pos == 4);
  HOTSPOT_MODE      = (pos == 5);
}

void testMosfet() {
  // MOSFET kurz aus- und wieder einschalten. Dient als Test der
  // Selbsthalte-Schaltung: Fällt die Versorgung weg, geht der ESP sauber aus.
  gpio_set_level((gpio_num_t)MOSFET, 0);
  vTaskDelay(pdMS_TO_TICKS(100));
  gpio_set_level((gpio_num_t)MOSFET, 1);
}

