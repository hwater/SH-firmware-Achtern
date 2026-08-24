// Neustart, wenn der Signal-K-WebSocket dauerhaft tot bleibt.
//
// WARUM DAS HIER STEHT UND NICHT IN SENSESP:
// SensESP bringt enable_wifi_watchdog() mit, der aber ausschliesslich auf
// kWifiDisconnected / kWifiNoAP reagiert — also nur auf das, was der WLAN-Stack
// selbst zugibt. Am 15.08.2026 hing der Mast-Kompass am Strom (Ankerlicht an)
// und war trotzdem nicht erreichbar, waehrend sein Stack "verbunden" meldete.
// In Signal K stand einfach ein Wert von vor zwei Stunden — nichts an dieser
// Situation sieht nach Fehler aus, und niemand bemerkt sie.
// Perkins und AchternSensorik rufen enable_wifi_watchdog() ohnehin nicht auf,
// haben also gar keinen Reboot-Wachhund.
//
// Diese Datei liegt bewusst im Projekt und nicht als Patch in .pio/libdeps:
// Bibliotheks-Patches ueberleben kein Update (siehe die n2k-signalk-Patches
// auf dem Pi, die bei jedem Server-Update neu eingespielt werden muessen).
//
// Massgeblich ist der WS-Zustand, nicht WiFi.status(). Der SystemStatusController
// verteilt kSKWSConnected/kSKWSConnecting/kSKWSDisconnected ohnehin schon.
#pragma once

#include <Arduino.h>

#include "sensesp/controllers/system_status_controller.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp_app.h"

// So lange ohne stehenden WebSocket -> harter Neustart.
#ifndef WS_REBOOT_S
#define WS_REBOOT_S 600UL
#endif

/**
 * @brief Startet den WS-Neustart-Wachhund. NACH get_app() aufrufen.
 *
 * Bewusst NICHT ueber Debounce<SystemStatus> geloest (so macht es SensESPs
 * eigener wifi_watchdog): flippt der Client im Sekundentakt zwischen
 * kSKWSDisconnected und kSKWSConnecting, wird ein Debounce nie stabil und
 * feuert nie. Stattdessen ein Zeitstempel, der nur bei gutem Zustand
 * nachgezogen wird — das ist immun gegen Flattern.
 *
 * kSKWSAuthorizing gilt als "gut": laeuft gerade ein Access-Request, muss der
 * Zugang im Admin-UI von Hand freigegeben werden. Das dauert laenger als jedes
 * Timeout, und ein Neustart mittendrin wuerde die Anfrage verwerfen.
 */
inline void start_ws_reboot_watchdog(uint32_t timeout_s = WS_REBOOT_S) {
  static sensesp::SystemStatus last_status =
      sensesp::SystemStatus::kSKWSDisconnected;
  static uint32_t ws_ok_ms = millis();

  sensesp::sensesp_app->get_system_status_controller()->connect_to(
      new sensesp::LambdaConsumer<sensesp::SystemStatus>(
          [](sensesp::SystemStatus s) { last_status = s; }));

  sensesp::SensESPBaseApp::get_event_loop()->onRepeat(10000, [timeout_s]() {
    if (last_status == sensesp::SystemStatus::kSKWSConnected ||
        last_status == sensesp::SystemStatus::kSKWSAuthorizing) {
      ws_ok_ms = millis();
      return;
    }
    if (millis() - ws_ok_ms > timeout_s * 1000UL) {
      Serial.printf("WS seit %lu s tot trotz WLAN — Neustart\n",
                    (unsigned long)timeout_s);
      Serial.flush();
      delay(120);
      ESP.restart();
    }
  });
}
