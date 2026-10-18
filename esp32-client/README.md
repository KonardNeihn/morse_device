# esp32-client – Morse-Gerät (Client)

Der ESP32-C5-Client des Morse-Projekts. Er nimmt Morse-Eingaben über eine Taste
auf, sendet sie an einen Server, empfängt Nachrichten, spielt sie über
Lautsprecher/LED ab und druckt sie auf einem Thermodrucker.

## Hardware

- Board: **ESP32-C5-DevKitC-1** (Single-Core RISC-V)
- Framework: **ESP-IDF 6.1.0** (nativ, kein Arduino/PlatformIO)

## Pins

| Funktion            | GPIO        |
|---------------------|-------------|
| Thermodrucker TX    | 4  (UART1)  |
| Thermodrucker RX    | 5  (UART1)  |
| Lautsprecher        | 8  (LEDC/PWM)|
| Morse-Taste         | 9  (LOW = gedrückt) |
| Status-LED          | 10          |
| MOSFET (Selbsthalte)| 23          |
| Drehschalter        | GPIO1 (ADC1_CH0, Spannungsteiler) |

## Bauen & Flashen

```bash
idf.py set-target esp32c5        # nur beim allerersten Mal nötig
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

## Drehschalter-Modi (6 Positionen)

| Position | Modus         | Bedeutung                         |
|----------|---------------|-----------------------------------|
| 1        | NORMAL        | Normalbetrieb                     |
| 2        | NO_SOUND      | Ton aus                           |
| 3        | NO_PRINTER    | Drucker aus                       |
| 4        | SELF_CHECK    | Selbsttest (ohne Netzwerk)        |
| 5        | SERVER_CHECK  | Paket zum Server und zurück       |
| 6        | HOTSPOT       | Konfigurations-AP „Morse"         |

## WLAN konfigurieren

1. Drehschalter auf **HOTSPOT** stellen.
2. Am Handy mit dem offenen AP **„Morse"** verbinden → das Captive Portal
   öffnet automatisch (sonst manuell `http://192.168.4.1/`).
3. WLAN 1 (und optional WLAN 2) eintragen und speichern.
   - Passwort nur eingeben, wenn es geändert werden soll (leer = behalten).
   - **„Offenes Netzwerk (kein Passwort)"** ankreuzen für Netze ohne Passwort.

## Server

- Hostname/Port stehen in `include/config.h` (`server_address`, `port`).
- Drahtformat: 11-Byte-Header (`status`, `msg_id` 64 Bit BE, `size` 16 Bit BE)
  + Payload – Details in `src/network.cpp`.

## Firmware-Version

- `FIRMWARE_VERSION` in `include/config.h` (wird beim Boot geloggt).
- Release-Stand per Git-Tag (`v1.0.0`).
