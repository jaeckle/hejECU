// src/SystemConfig.h
#pragma once


#include <Arduino.h>
#include <ArduinoJson.h>
#include "CoreDefs.h" // Für LogLevel

// =========================================================
// 1. ZENTRALE VARIABLEN
// =========================================================

// Das LogLevel ist eine der wenigen Variablen, die das gesamte System steuert
extern volatile LogLevel activeLogLevel;
bool loadWiFiConfig(JsonVariant json);

// =========================================================
// 2. FUNKTIONSDEKLARATIONEN (I/O Steuerung)
// =========================================================

// Definition der Hauptkonfigurationsstruktur
struct SystemConfig_t {
    char wifi_ssid[32];
    char wifi_password[64];
    // ... andere globale Konfigurationen
};

extern SystemConfig_t systemConfig; // Die externe Instanz

// Funktionsdeklarationen
bool loadConfig();
bool saveConfig();
bool loadDataFromJson(const char *jsonString);

// Modul-übergreifende Load/Save Funktionen (intern vom Manager genutzt)
// Diese Funktionen rufen die load/save Methoden in den einzelnen Managern auf.
bool loadModuleConfigs(JsonVariant json);
void saveModuleConfigs(JsonObject json);
