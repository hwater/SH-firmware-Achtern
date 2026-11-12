// Warum das Geraet zuletzt neu gestartet ist — und das Geraet sagt es selbst.
//
// WARUM:
// "ACHTERNS hat 8 mal neu gestartet" liess sich bisher nur forensisch aus dem
// Heap-CSV auf dem Pi rekonstruieren (Uptime-Rueckspruenge), und selbst dann
// blieb offen, OB ein Wachhund zugeschlagen hat, WELCHER, oder ob das Geraet
// abgestuerzt / der Strom weg war. Das Geraet weiss das aber im Moment des
// Neustarts. Es soll den Grund hinterlegen und beim naechsten Boot deutlich
// nach Signal K melden (diagnostics.<geraet>.lastReset).
//
// WIE:
// Vor einem bewussten Neustart (WS- oder Heap-Wachhund) wird ein Grund-Code ins
// RTC-RAM gelegt. Das ueberlebt ESP.restart(), aber NICHT einen Brownout oder
// das Trennen vom Strom — und genau diese Unterscheidung ist die halbe Antwort:
// liegt beim Boot ein gueltiger Code vor, war es ein Wachhund; liegt keiner vor,
// verraet esp_reset_reason() den Rest (Kaltstart, Brownout, Absturz, Flash).
//
// Wie ws_reboot_watchdog.h / heap_watchdog.h bewusst im Projekt, nicht als
// Bibliotheks-Patch. Datei identisch in SH-firmware-Perkins und -Achtern.
#pragma once

#include <Arduino.h>
#include <esp_system.h>

#include "sensesp/sensors/sensor.h"          // RepeatSensor
#include "sensesp/signalk/signalk_output.h"  // SKOutputString

// Bewusste Neustart-Gruende. 0 heisst "von uns nichts hinterlegt" — dann war
// es kein Wachhund, sondern Strom/Absturz/Flash (siehe esp_reset_reason()).
enum RebootGrund : uint32_t {
  REBOOT_KEINER = 0,
  REBOOT_WS_HAENGER = 1,   // WS stand, brach weg: Haenger trotz WLAN "verbunden"
  REBOOT_WS_NETZ_WEG = 2,  // seit Boot nie verbunden: Netz selbst fehlt
  REBOOT_HEAP = 3,         // Heap unter die Wachhund-Schwelle gefallen
};

// RTC-RAM: ueberlebt ESP.restart() und Deep-Sleep, ist nach einem Kaltstart
// aber Zufall. Das Magic-Wort daneben sagt, ob der Grund-Code gueltig ist.
static const uint32_t kRebootMagic = 0x57420001UL;  // "WB" + Version
RTC_NOINIT_ATTR static uint32_t g_reboot_magic;
RTC_NOINIT_ATTR static uint32_t g_reboot_grund;

/** Vor einem bewussten ESP.restart() aufrufen. */
inline void set_reboot_grund(RebootGrund grund) {
  g_reboot_magic = kRebootMagic;
  g_reboot_grund = (uint32_t)grund;
}

/**
 * @brief Ermittelt den Klartext des letzten Neustarts EINMAL und invalidiert
 *        danach den gespeicherten Grund, damit der naechste (dann evtl.
 *        externe) Neustart nicht faelschlich denselben Wachhund-Grund zeigt.
 */
inline String reboot_grund_ermitteln() {
  const bool wachhund = (g_reboot_magic == kRebootMagic);
  const uint32_t grund = wachhund ? g_reboot_grund : REBOOT_KEINER;
  g_reboot_magic = 0;  // verbraucht

  if (wachhund) {
    switch (grund) {
      case REBOOT_WS_HAENGER:  return F("WS-Wachhund: WS haengt (WLAN ok)");
      case REBOOT_WS_NETZ_WEG: return F("WS-Wachhund: Netz weg");
      case REBOOT_HEAP:        return F("Heap-Wachhund: Speicher knapp");
      default: break;
    }
  }
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:   return F("Kaltstart (Strom an)");
    case ESP_RST_BROWNOUT:  return F("Spannungseinbruch (Brownout)");
    case ESP_RST_PANIC:     return F("Absturz (Panic/Exception)");
    case ESP_RST_INT_WDT:   return F("System-Watchdog (Interrupt)");
    case ESP_RST_TASK_WDT:  return F("System-Watchdog (Task)");
    case ESP_RST_WDT:       return F("System-Watchdog");
    case ESP_RST_SW:        return F("Software-Reset (Flash/API)");
    case ESP_RST_DEEPSLEEP: return F("aus Deep-Sleep");
    case ESP_RST_EXT:       return F("externer Reset-Pin");
    default:                return String(F("unbekannt (")) +
                                   (int)esp_reset_reason() + F(")");
  }
}

/**
 * @brief Meldet den Grund des letzten Neustarts dauerhaft nach Signal K.
 *
 * NACH get_app() aufrufen. Der Wert wird alle 60 s gesendet, damit er nach
 * jedem Server-Neustart wieder im Baum steht und jederzeit abrufbar ist — eine
 * einmalige Delta beim Boot waere nach einem SK-Neustart verschwunden.
 *
 * @param sk_pfad     z.B. "diagnostics.achtern.lastReset"
 * @param meta_pfad   z.B. "/diagnostics/achtern/lastReset"
 */
inline void start_reboot_reason_report(const char* sk_pfad,
                                       const char* meta_pfad) {
  static String grund = reboot_grund_ermitteln();
  Serial.printf("Letzter Neustart-Grund: %s\n", grund.c_str());

  auto* quelle = new sensesp::RepeatSensor<String>(60000, []() -> String {
    return grund;
  });
  quelle->connect_to(new sensesp::SKOutputString(sk_pfad, meta_pfad));
}
