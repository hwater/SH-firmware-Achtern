// Signal-K-Identitaet = Hostname statt zufaelliger UUID.
//
// WARUM:
// SensESP wuerfelt beim ersten Access-Request eine UUID als clientId
// (generate_uuid4() in SKWSClient::send_access_request()) und merkt sie sich
// in SPIFFS. Signal K fuehrt das Geraet danach unter genau dieser ID — in
// Security -> Devices steht dann "eeac5ae3-6bf3-4acd-b33e-6fa4cb241e40" statt
// eines Namens, und nach jedem Werksreset kommt eine neue UUID dazu. Welches
// Board welches ist, laesst sich nur noch ueber die Beschreibung erraten.
//
// WAS PASSIERT:
// Weicht die gespeicherte clientId vom Hostnamen ab, wird sie auf den Hostnamen
// gesetzt und das alte Token verworfen — es ist an die alte ID gebunden, der
// Server wuerde das Geraet damit weiter unter der UUID fuehren. SensESP stellt
// beim naechsten Verbindungsaufbau einen neuen Access-Request, der EINMAL im
// Signal-K-Admin (Security -> Access Requests) freigegeben werden muss. Den
// alten UUID-Eintrag unter Security -> Devices danach loeschen.
//
// Aendert jemand den Hostnamen in der Web-Oberflaeche, folgt die clientId beim
// naechsten Boot nach — auch das kostet eine erneute Freigabe.
//
// Die Priorities in Signal K haengen NICHT an der clientId, sondern am
// $source-Label der Deltas; die bleiben von der Umstellung unberuehrt.
//
// Wie ws_reboot_watchdog.h bewusst im Projekt statt als Bibliotheks-Patch.
// Datei ist identisch in SH-firmware-Perkins und SH-firmware-Achtern.
#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>

#include "sensesp_app.h"

/**
 * @brief Legt die Signal-K-clientId auf den Hostnamen fest.
 *
 * NACH get_app() und NACH dem Setzen des Hostnamens aufrufen, aber noch in
 * setup(): der WebSocket-Client verbindet sich erst aus dem Event-Loop heraus,
 * die neue ID ist dann schon gespeichert.
 */
inline void pin_sk_client_id(const String& want) {
  auto ws = sensesp::sensesp_app->get_ws_client();
  if (!ws) return;
  if (want.isEmpty()) return;

  JsonDocument doc;
  JsonObject cfg = doc.to<JsonObject>();
  ws->to_json(cfg);

  const String have = cfg["client_id"].as<String>();
  if (have == want) return;

  cfg["client_id"] = want;
  cfg["token"] = "";         // gehoert zur alten ID
  cfg["polling_href"] = "";  // offene Anfrage der alten ID nicht weiterpollen
  ws->from_json(cfg);
  ws->save();

  Serial.printf("Signal-K-clientId: '%s' -> '%s' (Freigabe im SK-Admin noetig)\n",
                have.c_str(), want.c_str());
}

/** clientId = Hostname (Standardfall). */
inline void pin_sk_client_id_to_hostname() {
  pin_sk_client_id(sensesp::SensESPBaseApp::get_hostname());
}
