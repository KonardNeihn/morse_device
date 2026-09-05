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
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "app_state.h"
#include "config.h"

static const char* TAG = "hardware";

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

  // --- Eingänge: Drehschalter (extern hochgezogen, LOW = aktiv). ---
  gpio_config_t in = {};
  in.pin_bit_mask = (1ULL << NORMAL_MODE_PIN) | (1ULL << NO_SOUND_MODE_PIN) |
                    (1ULL << NO_PRINTER_MODE_PIN) | (1ULL << SELF_CHECK_MODE_PIN) |
                    (1ULL << SERVER_CHECK_MODE_PIN) | (1ULL << RICK_ROLL_MODE_PIN);
  in.mode = GPIO_MODE_INPUT;
  in.pull_up_en = GPIO_PULLUP_DISABLE;
  in.pull_down_en = GPIO_PULLDOWN_DISABLE;
  in.intr_type = GPIO_INTR_DISABLE;
  ESP_ERROR_CHECK(gpio_config(&in));

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
  // Drehschalter sind LOW-aktiv: Pin = 0 bedeutet "Modus aktiv".
  // gpio_get_level() liest den aktuellen Pegel eines Pins (0 = LOW, 1 = HIGH).
  NO_SOUND_MODE = (gpio_get_level((gpio_num_t)NO_SOUND_MODE_PIN) == 0);
  NO_PRINTER_MODE = (gpio_get_level((gpio_num_t)NO_PRINTER_MODE_PIN) == 0);
  SELF_CHECK_MODE = (gpio_get_level((gpio_num_t)SELF_CHECK_MODE_PIN) == 0);
  SERVER_CHECK_MODE = (gpio_get_level((gpio_num_t)SERVER_CHECK_MODE_PIN) == 0);
  RICK_ROLL_MODE = (gpio_get_level((gpio_num_t)RICK_ROLL_MODE_PIN) == 0);
}

void testMosfet() {
  // MOSFET kurz aus- und wieder einschalten. Dient als Test der
  // Selbsthalte-Schaltung: Fällt die Versorgung weg, geht der ESP sauber aus.
  gpio_set_level((gpio_num_t)MOSFET, 0);
  vTaskDelay(pdMS_TO_TICKS(100));
  gpio_set_level((gpio_num_t)MOSFET, 1);
}

