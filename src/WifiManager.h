// src/WiFiManager.h (KORREKTUREN)
#pragma once

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

// Externe Variable, die den AP-Modus steuert (in WiFiManager.cpp definiert)
extern bool startApMode;

/**
 * @brief Versucht, eine Verbindung zum konfigurierten STA-Netzwerk herzustellen.
 * @return true, wenn die Verbindung erfolgreich war (STA Mode); false sonst.
 */
bool setupWiFi(); // <-- HIER: void ZU bool GEÄNDERT

/**
 * @brief Verwaltet den WLAN-Zustand (STA/AP). Muss periodisch im Loop aufgerufen werden.
 */
void handleWiFiMode(); // <-- HIER: HINZUGEFÜGT

/**
 * @brief Definiert und startet den Access Point.
 */
void startAPMode();