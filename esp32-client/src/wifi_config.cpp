// =============================================================================
// wifi_config.cpp  –  persistente WLAN-Zugangsdaten (NVS)
// =============================================================================

#include "wifi_config.h"

#include <esp_log.h>
#include <nvs.h>

#include <string.h>

static const char* TAG = "wifi_config";

// NVS-Namespace und Schlüssel für die vier Felder.
static constexpr const char* NVS_NS = "wifi";
static constexpr const char* KEY_SSID1 = "ssid1";
static constexpr const char* KEY_PW1   = "pw1";
static constexpr const char* KEY_SSID2 = "ssid2";
static constexpr const char* KEY_PW2   = "pw2";

// Maximale Längen (passend zu wifi_config_t: SSID max. 32, Passwort max. 64).
static constexpr size_t SSID_MAX = 32;
static constexpr size_t PASS_MAX = 64;

// Aktuell geladene Werte (auch vom HTTP-Handler lesbar).
static char ssid1[SSID_MAX + 1] = { 0 };
static char pw1[PASS_MAX + 1]   = { 0 };
static char ssid2[SSID_MAX + 1] = { 0 };
static char pw2[PASS_MAX + 1]   = { 0 };

static void setBuf(char* dst, size_t cap, const char* src) {
  if (src != nullptr)
    strlcpy(dst, src, cap);
  else
    dst[0] = '\0';
}

static void readStr(nvs_handle_t h, const char* key, char* out, size_t cap) {
  size_t len = cap;
  if (nvs_get_str(h, key, out, &len) != ESP_OK)
    out[0] = '\0';
}

void loadWifiConfig() {
  nvs_handle_t h;
  if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) {
    ESP_LOGI(TAG, "keine gespeicherte WLAN-Konfiguration gefunden");
    return;
  }

  readStr(h, KEY_SSID1, ssid1, sizeof(ssid1));
  readStr(h, KEY_PW1, pw1, sizeof(pw1));
  readStr(h, KEY_SSID2, ssid2, sizeof(ssid2));
  readStr(h, KEY_PW2, pw2, sizeof(pw2));
  nvs_close(h);

  ESP_LOGI(TAG, "WLAN-Konfiguration geladen (ssid1=%s)", ssid1);
}

bool saveWifiConfig(const char* s1, const char* p1,
                    const char* s2, const char* p2) {
  // In-Memory sofort aktualisieren -> ohne Neustart nutzbar.
  setBuf(ssid1, sizeof(ssid1), s1);
  setBuf(pw1, sizeof(pw1), p1);
  setBuf(ssid2, sizeof(ssid2), s2);
  setBuf(pw2, sizeof(pw2), p2);

  nvs_handle_t h;
  if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
    ESP_LOGE(TAG, "NVS öffnen fehlgeschlagen");
    return false;
  }

  auto write = [&](const char* key, const char* val) {
    if (val != nullptr && val[0] != '\0')
      nvs_set_str(h, key, val);
    else
      nvs_erase_key(h, key);
  };
  write(KEY_SSID1, ssid1);
  write(KEY_PW1, pw1);
  write(KEY_SSID2, ssid2);
  write(KEY_PW2, pw2);

  esp_err_t err = nvs_commit(h);
  nvs_close(h);

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "NVS commit fehlgeschlagen");
    return false;
  }

  ESP_LOGI(TAG, "WLAN-Konfiguration gespeichert (ssid1=%s)", ssid1);
  return true;
}

const char* getSsid1()     { return ssid1; }
const char* getPassword1() { return pw1; }
const char* getSsid2()     { return ssid2; }
const char* getPassword2() { return pw2; }

bool hasWifiConfig() { return ssid1[0] != '\0'; }
