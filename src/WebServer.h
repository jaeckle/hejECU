// src/WebServer.h (Reiner Router/Handler Deklaration)
#pragma once
#include <Arduino.h>
#include <ESP8266WebServer.h>
#include "WiFiManager.h" // NEU: Für die korrekte Deklaration von apPassword etc.
#include "AppLogger.h"

// Externe Deklaration der Webserver-Instanz
extern ESP8266WebServer server;

// =========================================================
// 1. HAUPT-FUNKTIONEN
// =========================================================

void setup_server_routes();
void setup_ota();
bool isAuthorized(String user, String pass);

void setServerStarted(bool state);

// =========================================================
// 2. HANDLER-DEKLARATIONEN (HTTP-Endpunkte)
// =========================================================

// Haupt-Handler
void handleRoot();
void handleNotFound();
void handleLogin();
void handleLogout();
void handleSystemStatus();

// Konfigurations-Handler
void handleConfig();
void handleSaveConfig();
void handleRpmConfig();
void handleSaveRpmConfig();
void handleMapConfig();
void handleSaveMapConfig();

// Sensoren-Handler
void handleSensorConfig();
void handleSaveSensorConfig();

// =========================================================
// 3. HTML HILFSFUNKTIONEN (Delegation)
//    Diese Funktionen müssen in WebServer.cpp LEER oder MINIMAL implementiert sein.
//    Die eigentliche Generierung erfolgt in den Managern.
// =========================================================

String getHeader(String title);
String getFooter();
String getNavigation();
String generateStatusTable();
String generateIgnitionConfigForm();
// ... (alle anderen generierenden Funktionen)