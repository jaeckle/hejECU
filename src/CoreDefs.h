/*
 * Copyright (c) 2025 "hej und Gemini" (Contributor: [Ihr GitHub-Username])
 *
 * This software is licensed under the GNU General Public License v3.0 (GPL-3.0).
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation.
 *
 * A copy of the license is provided in the LICENSE file in the project root.
 */

 // CoreDefs.h
#ifndef CORE_DEFS_H
#define CORE_DEFS_H

#include <Arduino.h>
#include <vector>

// --- GLOBALE LOG-LEVELS ---
enum LogLevel { LOG_DEBUG, LOG_INFO, LOG_WARN, LOG_ERROR };

// Forward Deklaration des Loggers
void app_log(LogLevel level, const char* file, int line, int logLine, const char* format, ...);
// Das Makro muss hier sein, um in beiden .cpp Dateien LOG(LOG_...) zu ermöglichen
#define LOG(level, format, ...) app_log(level, __FILE__, __LINE__, __LINE__, format, ##__VA_ARGS__) 

// --- RPM Konfiguration ---
struct RpmConfig {
    volatile int pulses_per_revolution; 
    volatile bool is_digital; 
    volatile int input_pin;
}; 

// --- Zündungs-Timing Konfiguration ---
struct IgnitionTimingConfig {
    volatile int trigger_offset_deg; 
    volatile float TDC_adjust_deg;  
}; 

// --- Zündspulen-Konfiguration ---
struct IgnitionCoilConfig {
    volatile float primary_resistance_ohm;  
    volatile float external_resistance_ohm; 
    volatile float primary_inductance_mH;   
    volatile float target_current_A;        
    volatile float fixed_dwell_ms;          
};

// --- Zündkennfeld Struktur ---
struct MapPoint { int rpm; int tps; int angle; };

// Externe Deklaration der kritischen Variablen (definiert in main.cpp)
extern volatile LogLevel activeLogLevel;
extern bool serverStarted; 
extern String wifiSsid;
extern String wifiPassword;
extern String apSsid;
extern String apPassword;

extern struct RpmConfig rpmConfig;
extern struct IgnitionTimingConfig timingConfig;
extern struct IgnitionCoilConfig coilConfig;
extern std::vector<MapPoint> ignitionMap;
extern std::vector<String> startupLogBuffer;
extern const char *LOG_FILENAME; 

// KORRIGIERT: Hinzufügen von 'volatile'
extern volatile unsigned long max_loop_duration_us; // HINZUGEFÜGT: volatile
// Externe Deklaration der Statistik-Variablen für handleRoot()
extern volatile bool is_critical_latency_active;
extern volatile unsigned long last_critical_max_duration_us;
extern volatile unsigned long last_critical_timestamp_ms;

// Externe Deklaration der kritischen Funktionen (definiert in main.cpp)
extern bool saveConfig();
extern bool isAuthorized();
extern int getIgnitionRpmSafe();
extern int getAdvanceAngle2D(int rpm, int tps);
extern int pinNameToCode(String pinName);
extern void setPinState(int pinCode, bool targetState);

// NEUE SICHERE GETTER FÜR SENSORSTATE
extern float getSensorTpsRawSafe();
extern float getSensorBattVSafe();
extern float getSensorTempKopfSafe();
extern float getSensorSpeedSafe();

#endif // CORE_DEFS_H