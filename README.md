# AchternSensorik - Wellendrehzahl & Richtungserkennung

## Übersicht

ESP32-basiertes Sensorsystem für Marine-Anwendungen zur Bestimmung von Wellendrehzahl und Drehrichtung mittels eines Hall-Sensors und 3 Magneten.

## Messprinzip: Drehrichtungserkennung

Drei Magnete sind außen an der Welle befestigt mit **ungleichmäßigen Abständen (1:2:4)**:

```
        Welle (Draufsicht)
        
            M1 (0°)
           /
    ------●----------
   |      |    1     |
   |      |          |
   |  4   ●  M2     |   Gesamtumfang = 7 Einheiten
   |      |  (51°)   |   1 Einheit = 51.4°
   |      |          |
   |      |    2     |
    ------●----------
          M3 (154°)
```

**Vorwärtsdrehung (CW):** Intervallfolge kurz → mittel → lang (1:2:4)  
**Rückwärtsdrehung (CCW):** Intervallfolge lang → mittel → kurz (4:2:1)

Der Algorithmus erkennt die Drehrichtung durch Vergleich der zeitlichen Abstände zwischen aufeinanderfolgenden Impulsen.

## Pin-Belegung

| GPIO | Funktion                    |
|------|-----------------------------|
| 27   | Hall-Sensor (Engine RPM)    |
| 5    | CAN TX (NMEA2000)           |
| 4    | CAN RX (NMEA2000)           |
| 13   | OneWire Bus (DS18B20 x4)    |
| 21   | I2C SDA (BME680 + OLED)     |
| 22   | I2C SCL (BME680 + OLED)     |
| 34   | ADC RUDDER Voltage 12v      |
| 35   | ADC ENGINE Oilpressure pa   |

## Ausgabekanäle

1. **NMEA2000** - PGN 127488 (Engine RPM), alle 500ms
	**NMEA2000** - PGN 127245 (RUDDER), alle 500ms
	**NMEA2000** - PGN 127489 (ENGINE Oil pressure,oil temperature, coolant temperature )
2. **WebServer** - Live-Dashboard unter `http://192.168.4.1`
3. **OLED Display** - 3 Seiten mit automatischem Wechsel (5s)
4. **Serial** - Textausgabe alle 1s (115200 Baud)

## diese librarys verwenden für ap und web und vorbereiten auf signalk : 
https://github.com/SignalK/SensESP

## für nmea2000: https://github.com/ttlappalainen/NMEA2000 bzw für ESP32


## WiFi Access Point

- **SSID:** `AchternSensorik`
- **Passwort:** `REDACTED-AP-PW`
- **IP:** `192.168.4.1`

## OTA Update

```bash
# PlatformIO
pio run -t upload --upload-port AchternSensorik.local

# Arduino IDE
# Werkzeuge → Port → AchternSensorik
# Passwort: REDACTED-OTA-PW
```

## Aufbau mit PlatformIO

```bash
cd AchternSensorik
pio run           # Kompilieren
pio run -t upload # Upload via USB
pio device monitor # Serielle Ausgabe
```

## Benötigte Hardware

- ESP32 DevKit
- Hall-Sensor (z.B. A3144, SS49E)
- 3x Neodym-Magnete
- SSD1306 OLED 128x64 (I2C)
- CAN-Transceiver (MCP2551 oder SN65HVD230)
- BME680 Breakout (optional)
- 1-4x DS18B20 Temperatursensoren
- 2x Spannungsteiler (100K/27K) bei 14V max




