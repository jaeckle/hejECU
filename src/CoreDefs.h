// CoreDefs.h

#ifndef COREDEFS_H
#define COREDEFS_H

#include <Arduino.h>
#include <vector>
#include <memory>
#include <stdarg.h>

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

// =========================================================
// 1. TYPEN UND ENUMS
// =========================================================

enum LogLevel {
    LOG_DEBUG = 0,
    LOG_INFO = 1,
    LOG_WARN = 2,
    LOG_ERROR = 3,
};

// NEUE 2D MAP STRUKTUR FÜR BILINEARE INTERPOLATION
// Die Map ist als Gitter definiert:
// angle_data[tps_index][rpm_index] = Zündwinkel
struct IgnitionMap2D {
    std::vector<int> rpm_axis;    // Die X-Achse (z.B. 0, 1000, 2000, ...)
    std::vector<int> tps_axis;    // Die Y-Achse (z.B. 0, 256, 512, 1023)
    std::vector<std::vector<int>> angle_data; // Die eigentlichen Winkelwerte [tps][rpm]
};

struct RpmConfig {
    int pulses_per_revolution;
    bool ignition_active;
    int input_pin;
};

struct IgnitionTimingConfig {
    int trigger_offset_deg;
    float tdc_offset_ms;
};

struct IgnitionCoilConfig {
    float primary_resistance_ohm;
    float external_resistance_ohm;
    float primary_inductance_mH;
    float target_current_A;
    float fixed_dwell_ms;
};


// =========================================================
// 2. EXTERN DEKLARIERTE GLOBALE VARIABLEN
// =========================================================

extern ESP8266WebServer server;
extern volatile LogLevel activeLogLevel;
extern String wifiSsid;
extern String wifiPassword;
extern String apSsid;
extern String apPassword;
extern bool startApMode;
extern const char *LOG_FILENAME;
extern std::vector<String> startupLogBuffer;
extern bool serverStarted;
extern volatile unsigned long max_loop_duration_us;
extern volatile bool is_critical_latency_active;
extern volatile unsigned long last_critical_max_duration_us;
extern volatile unsigned long last_critical_timestamp_ms;

extern struct RpmConfig rpmConfig;
extern struct IgnitionTimingConfig timingConfig;
extern struct IgnitionCoilConfig coilConfig;

// NEU: Alte ignitionMap (falls sie existierte) wurde entfernt.
extern IgnitionMap2D ignitionMap2D; // NEU: Unsere zentrale Kennfeld-Struktur

// =========================================================
// 3. EXTERN DEKLARIERTE FUNKTIONEN
// =========================================================

// Logger
extern void app_log(LogLevel level, const char* file, int line, int logLine, const char* format, ...);

// Getter (für sicheren Zugriff auf volatile Daten)
extern unsigned long getIgnitionPeriodSafe();
extern int getIgnitionRpmSafe();
extern unsigned long getAdvanceMicrosSafe();
extern float getSpeedStateSafe();
extern bool isAuthorized(); // Authentifizierungsfunktion

// SENSOR GETTER
extern float getSensorTpsRawSafe();
extern float getSensorBattVSafe();
extern float getSensorTempKopfSafe();
extern float getSensorSpeedSafe();

// Helfer
extern bool saveConfig();
extern bool loadDataFromJson(const char* jsonString);
extern const char* FALLBACK_CURVE_JSON; 
extern int pinNameToCode(String pinName);
extern void setPinState(int pinCode, bool targetState);
extern void setup_server_routes();
extern void setup_ota();
extern void logCriticalEvent(unsigned long maxDuration, unsigned long timestamp, const char* status);


#endif // COREDEFS_H