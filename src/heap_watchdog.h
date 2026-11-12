// Neustart, bevor der Heap aufgebraucht ist - und ein Messwert, der zeigt, ob
// er es wird.
//
// WARUM: Am 02.11.2026 stand der Achtern tagelang stumm an Signal K. Er war
// per Ping und HTTP erreichbar, seine Sensoren massen, die Weboberflaeche
// antwortete - nur der WebSocket war tot. Der Grund stand in /api/info:
//
//     Min free memory before last reset (bytes)   3836
//
// Mit nicht einmal vier Kilobyte bekommt der WebSocket-Client keinen Puffer
// mehr. Dabei ist der Heap nicht "voll", sondern zerstueckelt: Der ESP32
// kompaktiert nicht, und die Firmware legte pro Tag rund eine Million kurze
// String-Objekte an (13 Statuszeilen im Sekundentakt). Das ist die Ursache,
// und sie ist in main.cpp entschaerft - dieser Wachhund ist das Netz darunter.
//
// WAS ER TUT: alle 30 s den freien Heap ansehen. Faellt er unter die Schwelle,
// einen GEORDNETEN Neustart ausloesen, solange das billig ist - also bei
// stehender Welle. Ein Neustart ist allemal besser als ein Geraet, das
// scheinbar laeuft und nichts mehr meldet.
//
// WAS ER NICHT TUT, und das ist der wichtigere Teil:
//
//   * Nicht bei drehender Welle. Die AchternSensorik ueberwacht Oeldruck, Wellentemperatur und
//     Temperaturen; ein Neustart nimmt genau dann die Alarme weg, wenn sie
//     gebraucht werden. Unter Fahrt wird nur gewarnt - die CAN-Pfade laufen
//     ohnehin weiter (und an diesem Geraet haengt die Motorraum-
//     Gasueberwachung), und nach dem Abstellen greift der Wachhund.
//   * Nicht in den ersten Minuten nach dem Start. Waehrend WLAN, mDNS und der
//     Webserver hochkommen, ist der Heap kurzzeitig knapp; wer da zuschlaegt,
//     baut eine Reboot-Schleife.
//
// Dieselbe Begruendung wie bei ws_reboot_watchdog.h: Die Datei liegt im
// Projekt, nicht als Patch in .pio/libdeps - Bibliotheks-Patches ueberleben
// kein Update.
#pragma once

#include <Arduino.h>

#include "sensesp/system/lambda_consumer.h"
#include "sensesp_app.h"

#include "reboot_reason.h"

// Darunter wird neu gestartet (bei stehendem Motor).
#ifndef HEAP_REBOOT_BYTES
#define HEAP_REBOOT_BYTES 18000UL
#endif
// Darunter wird gewarnt - im Log und auf der Statusseite.
#ifndef HEAP_WARN_BYTES
#define HEAP_WARN_BYTES 35000UL
#endif
// So lange nach dem Start wird gar nichts unternommen.
#ifndef HEAP_GRACE_S
#define HEAP_GRACE_S 300UL
#endif

/**
 * @brief Startet den Heap-Wachhund. NACH get_app() aufrufen.
 *
 * @param rpm_quelle Zeiger auf die laufend aktualisierte Wellendrehzahl
 *        (sd.rpm). Bewusst ein
 *        Zeiger und kein Wert: der Wachhund muss die Drehzahl im Moment der
 *        Entscheidung lesen, nicht die beim Einrichten. Darf nullptr sein -
 *        dann gilt der Motor als stehend (so verhaelt sich der Wachhund auf
 *        einem Geraet ohne Drehzahlgeber).
 */
inline void start_heap_watchdog(const volatile float* rpm_quelle = nullptr) {
  static uint32_t start_ms = millis();
  static bool gewarnt = false;

  sensesp::SensESPBaseApp::get_event_loop()->onRepeat(30000, [rpm_quelle]() {
    const uint32_t frei = ESP.getFreeHeap();
    const uint32_t min_frei = ESP.getMinFreeHeap();

    if (frei < HEAP_WARN_BYTES && !gewarnt) {
      gewarnt = true;
      Serial.printf("Heap knapp: %lu Bytes frei (Minimum bisher %lu)\n",
                    (unsigned long)frei, (unsigned long)min_frei);
    } else if (frei > HEAP_WARN_BYTES + 8000UL) {
      gewarnt = false;          // wieder Luft: naechstes Mal erneut warnen
    }

    if (frei >= HEAP_REBOOT_BYTES) return;
    if (millis() - start_ms < HEAP_GRACE_S * 1000UL) return;

    float rpm = 0.0f;
    if (rpm_quelle != nullptr) {
      rpm = *rpm_quelle;
      if (isnan(rpm) || isinf(rpm)) rpm = 0.0f;
    }
    if (rpm >= 1.0f) {
      Serial.printf("Heap bei %lu Bytes, aber die Welle dreht (%.0f U/min) - "
                    "kein Neustart, die Alarme bleiben an\n",
                    (unsigned long)frei, rpm);
      return;
    }

    set_reboot_grund(REBOOT_HEAP);
    Serial.printf("Heap nur noch %lu Bytes frei - geordneter Neustart\n",
                  (unsigned long)frei);
    Serial.flush();
    delay(150);
    ESP.restart();
  });
}
