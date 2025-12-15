// src/WebServer.cpp (Reiner Router/Handler Implementierung)

#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>
#include <string.h>          // Für strlcpy
#include "WebServer.h"
#include "WiFiManager.h"
#include "SystemConfig.h"    // Für load/saveConfig
#include "IgnitionManager.h" // Für rpmConfig, timingConfig etc.
#include "SensorManager.h"   // Für Sensordaten

// =========================================================
// 1. GLOBALE VARIABLEN DEFINITIONEN
// =========================================================

// Globale Variablen (Deklariert in main.cpp)
extern bool startApMode;
extern bool serverStarted;

char wifiSsid[32] = "YOUR_WIFI_SSID";
char wifiPassword[64] = "YOUR_WIFI_PASSWORD";
char apSsid[32] = "Motor-AP";
char apPassword[64] = "12345678";

// =========================================================
// 3. HANDLER IMPLEMENTIERUNGEN
// =========================================================

void setupWiFi() { /* ... */ }
void handleWiFiClient() { /* ... */ }
bool loadWiFiConfig(JsonVariant json) { return true; } // Wichtig: Rückgabetyp beachten!

