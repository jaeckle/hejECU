// src/WiFiManager.h
#pragma once
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ArduinoJson.h> // Für die Load/Save Funktionen
#include "AppLogger.h"   // Für LOG Level

// =========================================================
// 1. GLOBALE VARIABLEN (Deklaration: Extern)
// WiFiManager besitzt jetzt diese Konfiguration
// =========================================================

// WLAN Anmeldedaten (als char[] zur Vermeidung von Heap-Fragmentierung)
extern char wifiSsid[];
extern char wifiPassword[];
extern char apSsid[];
extern char apPassword[];

// WLAN IP-Konfiguration (konstante Adressen)
extern const IPAddress apIP;
extern const IPAddress netMask;
extern const IPAddress gatewayIP;


// =========================================================
// 2. FUNKTIONSDEKLARATIONEN
// =========================================================

// Core Setup Funktionen
void setupWiFi();
void handleWiFiClient();
void startAccessPoint();

// Konfigurations-Handler (Neues Ownership-Modell: Jedes Modul lädt seine JSON-Teile selbst)
// Wird von SystemConfig.cpp aufgerufen
bool loadWiFiConfig(JsonVariant json);
void saveWiFiConfig(JsonObject json);