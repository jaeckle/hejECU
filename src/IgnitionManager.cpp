// src/IgnitionManager.cpp

#include "IgnitionManager.h"
#include "AppLogger.h"
#include <ArduinoJson.h>

// Deklarationen der IRAM-Handler aus main.cpp (Notwendig für attachInterrupt)
extern void IRAM_ATTR handleIgnitionPulse(); 
extern void IRAM_ATTR handleSpeedPulse();

// =========================================================
// 1. PIN DEFINITIONEN (DEFINITION)
// =========================================================
const int IGNITION_INPUT_PIN = D1;
const int SPEED_INPUT_PIN = D0;
const int IGNITION_OUTPUT_PIN = D2;
const int ADC_PIN = A0; 
const int LED_PIN = D4;
const int BUTTON_PIN = D5;

// =========================================================
// 2. GEKAPSELTE GLOBALE VARIABLEN (INTERNE DATENHALTUNG)
// =========================================================
// Diese Daten sind jetzt INTERN und nicht mehr extern!

// Konfigurations-Definitionen (Initialwerte)
RpmConfig rpmConfig = {
    .pulses_per_revolution = 1,     
    .ignition_enabled = true,       
    .input_pin = D1                 
};
IgnitionTimingConfig timingConfig = {
    .trigger_offset_deg = 360,      
    .timing_tdc = 0.0f              
};
IgnitionCoilConfig coilConfig = {
    .primary_resistance_ohm = 0.8f, 
    .external_resistance_ohm = 0.0f,
    .primary_inductance_mH = 4.0f,  
    .target_current_A = 8.0f,       
    .fixed_dwell_ms = 4.0f          
};
SpeedConfig speedConfig = {
    .sprocket_teeth = 15,           
    .wheel_circumference_mm = 2000  
};
IgnitionMap2D ignitionMap; 

// Definition der Laufzeit-Zustände (KEIN extern mehr!)
// WICHTIG: Diese Variablen sollten (wo nötig) als volatile deklariert werden, 
// da sie in Interrupts und im Loop verwendet werden.
extern volatile IgnitionState ignition; 
extern volatile SpeedState speedState; 


// =========================================================
// 3. SERVICE IMPLEMENTIERUNGEN
// =========================================================

// Schnittstellen-Implementierungen
IgnitionState getIgnitionState() { 
    // SICHERE KOPIE: Daten Member für Member kopieren
    IgnitionState copy;
    
    // Die Zuweisung Member für Member ist die einzige sichere Methode
    // um volatile Daten auszulesen, ohne den Copy-Konstruktor-Fehler auszulösen.
    copy.last_pulse_micros = ignition.last_pulse_micros;
    copy.period_micros = ignition.period_micros;
    copy.advance_micros = ignition.advance_micros;
    copy.rpm = ignition.rpm;
    copy.pulse_detected = ignition.pulse_detected;
    
    return copy; 
}

SpeedState getSpeedState() { 
    // SICHERE KOPIE: Daten Member für Member kopieren
    SpeedState copy;

    // Explizite Zuweisung der Member
    copy.last_pulse_micros = speedState.last_pulse_micros;
    copy.period_micros = speedState.period_micros;
    // ... (Fügen Sie hier alle anderen Member von SpeedState hinzu)
    
    return copy; 
}

RpmConfig getRpmConfig() { 
    return rpmConfig; 
}
// Initialisiert Pins/Interrupts (aus main.cpp verschoben)
void setupIgnition()
{
    LOG(LOG_INFO, "IgnitionManager: Initialisiere Pins und Interrupts.");
    
    // Initialisierung der GPIOs 
    pinMode(IGNITION_INPUT_PIN, INPUT_PULLUP);
    pinMode(IGNITION_OUTPUT_PIN, OUTPUT);
    pinMode(SPEED_INPUT_PIN, INPUT_PULLUP);
    pinMode(LED_PIN, OUTPUT); 
    pinMode(BUTTON_PIN, INPUT_PULLUP); 
    pinMode(ADC_PIN, INPUT); 
    
    digitalWrite(IGNITION_OUTPUT_PIN, LOW);
    digitalWrite(LED_PIN, LOW);

    // Interrupts setzen 
    timer1_disable();
    attachInterrupt(digitalPinToInterrupt(IGNITION_INPUT_PIN), handleIgnitionPulse, RISING);
    attachInterrupt(digitalPinToInterrupt(SPEED_INPUT_PIN), handleSpeedPulse, RISING);
}

void updateIgnitionParameters()
{
    // Logik wird später gefüllt
}

void updateSpeedCalibrationFactor()
{
    // Logik wird später gefüllt
}

bool loadIgnitionConfig(JsonVariant json) 
{
    // Logik wird später gefüllt
    return true; 
}

void saveIgnitionConfig(JsonObject json) 
{
    // Logik wird später gefüllt
}

// JSON Interface Implementierungen
void getIgnitionStateJson(JsonObject& doc) {
    // Lesen der volatile Variablen mit Cast
    IgnitionState currentState = getIgnitionState(); 
    doc["rpm"] = currentState.rpm;
    doc["period"] = currentState.period_micros;
    doc["advance"] = currentState.advance_micros;
    // ...
}

void getSpeedStateJson(JsonObject& doc) {
    // Lesen der volatile Variablen mit Cast
    SpeedState currentState = getSpeedState();
    doc["speed_period"] = currentState.period_micros;
    // ...
}