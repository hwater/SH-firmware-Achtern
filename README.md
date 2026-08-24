# SH-firmware-Achtern — AchternSensorik

SensESP firmware for the **AchternSensorik** unit (ESP32). It measures propeller
**shaft speed and rotation direction** (one Hall sensor + 3 magnets at 1:2:4
spacing) plus engine and environment sensors, and publishes everything over
**NMEA 2000** and **Signal K** with a live web dashboard.

## Drehrichtungs-Messprinzip (shaft direction)

Three magnets on the shaft at **unequal spacing (1 : 2 : 4)**:

```
            M1 (0°)
    ------●----------
   |      |    1     |
   |  4   ●  M2 (51°)|   Umfang = 7 Einheiten (1 Einheit ≈ 51.4°)
   |      |    2     |
    ------●----------
          M3 (154°)
```

- **Vorwärts (CW):** Intervallfolge kurz → mittel → lang (1:2:4)
- **Rückwärts (CCW):** Intervallfolge lang → mittel → kurz (4:2:1)

Direction is recognised from the ordering of the pulse intervals; RPM from the
total time of the three pulses. It can be inverted in the web UI
("Wellendrehrichtung → Richtung umdrehen") if the sensor is mounted the other way.

## Pin-Belegung

| GPIO | Funktion |
|------|----------|
| 27 | Hall-Sensor (Wellendrehzahl + Richtung) |
| 34 | ADC Ruder-Spannung → Winkel |
| 35 | ADC Öldruck |
| 13 | 1-Wire (4× DS18B20: Kühlwasser, Öl, Maschinenraum, Abgas) |
| 21 / 22 | I²C SDA / SCL (BME680 + SSD1306 OLED) |
| 4 / 5 | CAN RX / TX (NMEA 2000) |

## NMEA 2000 / Signal K
This board sends its engine PGNs on **engine instance 1** → Signal K
`propulsion.starboard`. **Instance 0 (`propulsion.port`) belongs to the Perkins
engine monitor** (`SH-firmware-Perkins`), which measures the engine itself; this
board only sees the propeller shaft, and the gearbox ratio makes its RPM a
different number. Do not move this board back onto instance 0 — the two boards
then overwrite each other's engine PGNs.

- PGN 127488 — shaft RPM (`propulsion.starboard.revolutions`)
- PGN 127245 — rudder angle (`steering.rudderAngle`)
- PGN 127489 — oil pressure + oil/coolant temperatures
  (`propulsion.starboard.*`). The engine-hours field stays `N2kDoubleNA`: this
  board has no hour meter, the Perkins monitor is the sole authority.
- BME680 → `environment.outside.{temperature,humidity,pressure,gasResistance}`

## Web interface (port 80)
- `/` — SensESP configuration UI (with a **Dash** navbar item)
- `/dash` — live dashboard: Welle · Motor & Ruder · Temperaturen · Umgebung ·
  NMEA 2000 · System
- `/status` — SensESP device status page (Sensoren + Canbus groups)
- `/api/data` — live JSON · `/api/can` — CAN controller diagnostics

In AP mode the unit is at `http://192.168.4.1` (SSID `AchternSensorik`); on a
joined network via `http://AchternSensorik.local`.

## Configuration (web UI → Configuration; persisted in flash)
- **Wellendrehrichtung** — "Richtung umdrehen" inverts forward/reverse (CW/CCW).
- **ADC Kalibrierung** — voltage-divider factors for rudder / oil pressure.
- **Temperatursensoren** — per DS18B20 (SensESP `OneWireTemperature`): assign the
  **1-Wire address**, set a **Linear calibration**, and the **SK path**; plus a
  display name. Addresses are auto-claimed on first boot and re-mappable here.
- **WLAN Access-Point** — auto-off after N minutes.

## Build & upload (PlatformIO, env `esp32dev`)
```
pio run -e esp32dev          # build
```
First flash via USB; afterwards OTA. The PlatformIO espota wrapper can stall, so
call espota.py directly with an explicit host IP (`-I`):
```
python3 ~/.platformio/packages/framework-arduinoespressif32/tools/espota.py \
  -i AchternSensorik.local -I <your-host-ip> -p 3232 \
  -a <ota_password> -f .pio/build/esp32dev/firmware.bin
```
The Wi-Fi AP password and OTA password live in the gitignored `src/secrets.h`
(copy `src/secrets.h.example` and fill it in). A pre-build script
(`scripts/patch_sensesp_navbar.py`) injects the **Dash** navbar entry, since
SensESP's route list is hardcoded and can't be extended from the sketch.

## Hardware
ESP32 DevKit · Hall sensor (A3144 / SS49E) · 3× neodymium magnets · SSD1306 OLED
128×64 (I²C) · CAN transceiver (SN65HVD230) · BME680 (optional) · 1–4× DS18B20 ·
2× voltage dividers (100K/27K) for the 12 V analog inputs.

---

## English translation

### Shaft-direction measurement principle

Three magnets on the shaft at **unequal spacing (1 : 2 : 4)**:

- **Forward (CW):** pulse interval sequence short → medium → long (1 : 2 : 4)
- **Reverse (CCW):** pulse interval sequence long → medium → short (4 : 2 : 1)

The direction is detected from the ordering of the pulse intervals; RPM is derived
from the total time of all three pulses. Direction can be inverted in the web UI
("Shaft direction → Invert direction") if the sensor is mounted the other way around.

### Pin assignment

| GPIO | Function |
|------|----------|
| 27 | Hall sensor (shaft speed + direction) |
| 34 | ADC rudder voltage → angle |
| 35 | ADC oil pressure |
| 13 | 1-Wire (4× DS18B20: coolant, oil, engine room, exhaust) |
| 21 / 22 | I²C SDA / SCL (BME680 + SSD1306 OLED) |
| 4 / 5 | CAN RX / TX (NMEA 2000) |

### Configuration (web UI → Configuration; stored in flash)

- **Shaft direction** — "Invert direction" swaps forward/reverse (CW/CCW).
- **ADC calibration** — voltage-divider factors for rudder and oil pressure inputs.
- **Temperature sensors** — per DS18B20 (SensESP `OneWireTemperature`): assign the
  **1-Wire address**, set a **Linear calibration**, and the **SK path**; plus a
  display name. Addresses are auto-claimed on first boot and re-mappable here.
- **Wi-Fi Access Point** — auto-off after N minutes.
