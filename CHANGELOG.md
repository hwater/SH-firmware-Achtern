# Changelog — SH-firmware-Achtern

SensESP firmware for the **AchternSensorik** unit (ESP32): propeller shaft
speed/direction plus engine and environment sensors, over NMEA 2000 and Signal K.
Most recent first.

## Sensors & configuration
- **DS18B20 temperatures via SensESP `OneWireTemperature`** (`d06b222`): each of
  the four sensors (Kühlwasser, Öl, Maschinenraum, Abgas) is assigned by its
  **1‑Wire address** in the web UI, with a **Linear calibration** and **SK path** —
  replacing the fragile bus‑index reading and the old per‑sensor offset. Addresses
  are auto‑claimed on first boot and re‑mappable in Configuration.
- **Shaft‑direction invert flag** ("Richtung umdrehen") on the Configuration
  page, for sensors mounted the other way around. (`b9df9f7`)

## NMEA 2000
- **Moved to engine instance 1** (Signal K `propulsion.starboard`): instance 0
  was shared with the Perkins engine monitor, so both boards' PGN 127488
  overwrote each other as "Engine 0 RPM" — and they are not the same number,
  since this board reads the propeller shaft and the Perkins board reads the
  engine (alternator W terminal), separated by the gearbox ratio. Instance 0
  now belongs to the Perkins monitor alone, keeping the actual engine on
  `propulsion.port` where displays expect it.
- **Stopped reporting this board's uptime as engine hours** (`dd8a3d3`): the
  engine‑hours field of **PGN 127489** carried `millis()/1000`. Because this
  board shares **engine instance 0** with the Perkins engine monitor, Signal K
  picked that uptime over the Perkins board's real hour meter —
  `propulsion.port.runTime` read 96.2 h (this board's uptime) instead of the
  engine's 1445.7 h. This board has no hour meter, so the field is now
  `N2kDoubleNA` and the Perkins monitor is the sole authority for engine hours.
  Note both boards still send **PGN 127488** (RPM) on instance 0.

## Stability
- **WiFi watchdog recovers from router outages** (v2.01): after a router
  outage on 2026‑07‑21 the WiFi stack hung for 21 h until a manual power
  cycle — SensESP's auto‑reconnect does not recover from e.g. an AP channel
  change after a router reboot. A 30‑second watchdog now forces a hard
  `disconnect()`/`reconnect()` (fresh scan) after 2 min offline, and restarts
  the device after 15 min offline — but only if WiFi was connected at least
  once since boot (no reboot loop while the router stays down) and the shaft
  is stopped (no N2K gap under way). N2K/CAN keeps running throughout; only
  the Signal K delta path depends on WiFi.
- **HTTP server no longer hangs after a few days uptime**: SensESP starts the
  ESP‑IDF `httpd` with `HTTPD_DEFAULT_CONFIG()` (`max_open_sockets = 7`,
  `lru_purge_enable = false`). Stale keep‑alive sockets from sleeping/departed
  browsers piled up until the listener could no longer accept connections and the
  web UI/API appeared frozen (~4 days). A pre‑build patch sets
  `lru_purge_enable = true` so the oldest session is purged to admit a new one —
  the server self‑heals. (`scripts/patch_sensesp_navbar.py`, Patch 4)

## Web interface
- **Perkins‑style `/dash`** (`4e6777c`): dark card grid, SensESP logo → config UI,
  live/offline status. Sensor cards (Welle, Motor & Ruder, Temperaturen, Umgebung)
  in green; **NMEA 2000** and **System** cards in white. Blue status badges and the
  restart button removed.
- **System card** — Hostname, IP, **Signal K** (replaces the old WLAN row),
  Laufzeit, and free heap. The Signal K row shows the live SK WebSocket client
  state (verbunden / verbindet… / autorisiert… / getrennt) via the new `sk`
  field in `/api/data`. (`4e6777c`, `04c7597`)
- **NMEA 2000 card + status "Canbus" group** — TX/RX Pakete, **TX/RX Fehler**
  (SJA1000 TXERR/RXERR registers, cached from the event loop), **Recoveries**, and
  the **device address** — matching Perkins. (`4e6777c`, `c7e455e`)
- **`/api/data`** extended with `ip`, `wifi`, `can_txerr`, `can_rxerr`,
  `can_recoveries`, `n2k_addr`, `free_heap`. (`4e6777c`, `c7e455e`, `04c7597`)
- **Dash** entry injected into the SensESP navbar (pre‑build script). (`b9df9f7`)
- Restart **403 Forbidden fix**: case‑insensitive origin check + AP‑SSID
  normalisation patches. (`8a789ec`)

## Repository & security
- Published as **SH-firmware-Achtern** with a **scrubbed history** (no credentials
  in any commit); Wi‑Fi/OTA secrets live in the gitignored `src/secrets.h`.
  (`119ab97`, `68564d4`)
- README documents I/O, web UI, configuration and OTA — German with an English
  translation section. (`d116750`)

## Operations
- **OTA** via `achternsensorik.local` (device `192.168.11.64`) using `espota.py`
  with an explicit host IP (`-I`). See the README.
- This repo is developed in parallel; integrate with `git fetch` + **rebase** onto
  `main` before pushing (never force‑push).
