// --- LIZENZ HINWEIS ---
/*
 * Copyright (c) 2025 "hej und Gemini" (Contributor: jaeckle)
 *
 * This software is licensed under the GNU General Public License v3.0 (GPL-3.0).
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation.
 *
 * A copy of the license is provided in the LICENSE file in the project root.
 */

#ifndef COREDEFS_H
#define COREDEFS_H

#include <Arduino.h>
#include <ESP8266WiFi.h> 
#include <ESP8266WebServer.h>
#include <vector> // Für std::vector<String> startupLogBuffer
#include <string> // Für std::vector<String> startupLogBuffer

// ************************************************************
// 1. SYSTEMKONSTANTEN
// ************************************************************

#define CORE_VERSION "0.0.100" 

#define IGNITION_INPUT_PIN D5
#define IGNITION_OUTPUT_PIN D6
#define SPEED_INPUT_PIN D7
#define ADC_PIN A0

#define WEB_HANDLE_INTERVAL 50   // ms
#define OTA_HANDLE_INTERVAL 1000 // ms

#define MAP_SIZE 8 // Größe der 2D-Zündwinkel-Tabelle

// ************************************************************
// 2. TYP-DEFINITIONEN (ENUMS & STRUCTS)
// ************************************************************

enum LogLevel {
    LOG_NONE = 0,
    LOG_ERROR,
    LOG_WARN,
    LOG_INFO,
    LOG_DEBUG
};

// Enum ergänzt um die fehlenden IDs aus den Fehlermeldungen in main.cpp
enum SensorID {
    SID_RPM = 0,
    SID_SPEED,
    SID_TPS_RAW,
    SID_TPS_CAL,
    SID_BATT_V,
    SID_TEMP_KOPF,
    SID_TEMP_LUFT,      // NEU: Aus Fehlerlog ergänzt
    SID_TEMP_UMG,       // NEU: Aus Fehlerlog ergänzt
    SID_FEUCHTIGKEIT,   // NEU: Aus Fehlerlog ergänzt
    SID_LIGHT,          // NEU: Aus Fehlerlog ergänzt
    SID_FARLIGHT,       // NEU: Aus Fehlerlog ergänzt
    SID_BLINK_L,        // NEU: Aus Fehlerlog ergänzt
    SID_BLINK_R         // NEU: Aus Fehlerlog ergänzt
};

struct RpmConfig {
    int pulses_per_revolution; 
    int max_rpm;               
};

struct SpeedConfig {
    int sprocket_teeth;
    int wheel_circumference_mm;
};

// NEU: Strukturen ergänzt, um 'incomplete type' Fehler in main.cpp (Zeile 73/74) zu beheben
struct IgnitionTimingConfig {
    int degrees;
    float factor;
    // Angenommene Felder, basierend auf Initialisierung {60, 0.0f}
};

struct IgnitionCoilConfig {
    float dwell_min_ms;
    float dwell_max_ms;
    float crank_dwell_ms;
    float max_spark_time_us;
    float safety_margin_ms;
    // Angenommene Felder, basierend auf Initialisierung {0.2f, 2.0f, 4.0f, 8.0f, 3.5f}
};


// ************************************************************
// 3. GLOBALE VARIABLEN (EXTERN DEKLARATIONEN)
// ************************************************************

// --- System und Konfiguration ---
extern LogLevel activeLogLevel;
extern std::vector<String> startupLogBuffer; // KORRIGIERT: Typ ist jetzt vector<String>
extern bool startApMode;
extern bool serverStarted;

// --- WLAN / Netzwerk ---
extern String wifiSsid;
extern String wifiPassword;
extern String apSsid;
extern String apPassword;

extern IPAddress apIP;      
extern IPAddress netMask;   
extern IPAddress gatewayIP; 

// --- Core Konfiguration ---
extern RpmConfig rpmConfig;
extern SpeedConfig speedConfig;

// NEU: Extern Deklarationen für die in main.cpp initialisierten structs
extern IgnitionTimingConfig timingConfig;
extern IgnitionCoilConfig coilConfig;

// --- Zündwinkel-Kennfeld-Daten ---
// Achtung: IgnitionMap2D wurde entfernt. Wir verwenden die Arrays.
extern int rpm_axis[MAP_SIZE];     
extern int tps_axis[MAP_SIZE];     
extern int angle_data[MAP_SIZE][MAP_SIZE];

// --- Laufzeitwerte (z.B. für handleRoot) ---
extern unsigned int rpm;
extern float tps;
extern float batteryVoltage;

// --- Webserver Instanz ---
extern ESP8266WebServer server;

#endif // COREDEFS_H