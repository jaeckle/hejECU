// src/SystemController.h

#pragma once

/**
 * @brief Führt alle initialen Setups (GPIO, LittleFS, WLAN, Ticker) aus.
 */
void setupModules();

/**
 * @brief Verwaltet die Hauptnetzwerk-Loops (Webserver, OTA, WLAN-Status).
 */
void handleNetworkLoop();

/**
 * @brief Führt periodische Systemwartungsaufgaben (Heap-Check, Statistiken) aus.
 */
void handleSystemMaintenance();