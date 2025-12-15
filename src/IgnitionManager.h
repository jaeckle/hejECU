// src/IgnitionManager.h - Service Interface

#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>   

// WICHTIG: Die Definitionen der structs wurden hierher von CoreDefs.h verschoben!
// Achten Sie darauf, sie aus CoreDefs.h zu löschen.

// =========================================================
// 1. PIN DEFINITIONEN (Bleiben extern)
// =========================================================
extern const int IGNITION_INPUT_PIN;
extern const int SPEED_INPUT_PIN;
extern const int IGNITION_OUTPUT_PIN;
extern const int ADC_PIN;
extern const int LED_PIN;
extern const int BUTTON_PIN;

// =========================================================
// 2. DATENSTRUKTUREN (VERSCHOBEN VON CoreDefs.h)
// =========================================================

// Konfiguration der Drehzahlerfassung und Zündungsaktivierung
struct RpmConfig {
    int pulses_per_revolution; 
    bool ignition_enabled;
    int input_pin;
};

// Zustand der Zündsteuerung (IRAM relevant)
struct IgnitionState {
    unsigned long last_pulse_micros;
    unsigned long period_micros;
    unsigned long advance_micros;
    int rpm;
    bool pulse_detected;
};

// Zustand der Geschwindigkeitserfassung (IRAM relevant)
struct SpeedState {
    unsigned long last_pulse_micros;
    unsigned long period_micros;
    // ... (Weitere Geschwindigkeitsmetriken)
};

// Weitere Structs aus CoreDefs.h
struct IgnitionTimingConfig {
    float trigger_offset_deg; 
    float timing_tdc;
};

struct IgnitionCoilConfig {
    float primary_resistance_ohm; 
    float external_resistance_ohm;
    float primary_inductance_mH;
    float target_current_A; 
    float fixed_dwell_ms;
};

struct SpeedConfig {
    int sprocket_teeth;
    int wheel_circumference_mm; 
};

struct IgnitionMap2D {
    int placeholder; 
};


// =========================================================
// 3. SERVICE FUNKTIONEN (Interface)
// =========================================================

void setupIgnition(); // Initialisiert Pins/Interrupts

// Zustands-Abfrage Funktionen (Gekapselter Zugriff)
IgnitionState getIgnitionState();
SpeedState getSpeedState();
RpmConfig getRpmConfig();

// Konfigurations-Funktionen
void updateIgnitionParameters(); // Berechnet den Zündzeitpunkt
void updateSpeedCalibrationFactor(); // Wird von SystemConfig aufgerufen

bool loadIgnitionConfig(JsonVariant json);
void saveIgnitionConfig(JsonObject json);

// NEU: JSON Interface für WebServer
void getIgnitionStateJson(JsonObject& doc);
void getSpeedStateJson(JsonObject& doc);