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

// =========================================================
// 2. FUNKTIONSDEKLARATIONEN (I/O Steuerung)
// =========================================================

// Haupt-I/O Funktionen (von main.cpp aufgerufen)
bool loadConfig();
bool saveConfig();
bool loadDataFromJson(const char *jsonString);

// Modul-übergreifende Load/Save Funktionen (intern vom Manager genutzt)
// Diese Funktionen rufen die load/save Methoden in den einzelnen Managern auf.
bool loadModuleConfigs(JsonVariant json);
void saveModuleConfigs(JsonObject json);