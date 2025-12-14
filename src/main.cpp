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
// main.cpp
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <Ticker.h>
#include <memory>
#include <vector>
#include <stdarg.h>
#include <algorithm>
#include <cmath>
#include <ESP8266mDNS.h>
#include <ESP8266WebServer.h>
#include <ArduinoOTA.h>

extern "C"
{
#include "user_interface.h"
}

// HEADER INKLUSIONEN
#include "CoreDefs.h"
#include "AppLogger.h" // NEU: AppLogger Modul
#include "WebServer.h"

#pragma GCC optimize("O3")

// =========================================================
// 1. GLOBALE DEFS UND LOGGER
// =========================================================

// --- ANDERE KONFIGURATION (Konstanten HIER DEFINIERT, um Sichtbarkeit zu gewährleisten) ---
const int LED_PIN = D4;
const int BUTTON_PIN = D5;
const char *HTTP_USERNAME = "admin";
bool webserverRunning = false;
IPAddress apIP(192, 168, 4, 1);      // <--- Definition mit Wert
IPAddress netMask(255, 255, 255, 0); // <--- Definition mit Wert
IPAddress gatewayIP(192, 168, 4, 1); // <--- Definition mit Wert

int rpm_axis[MAP_SIZE];
int tps_axis[MAP_SIZE];
int angle_data[MAP_SIZE][MAP_SIZE];

// --- WICHTIGE GLOBALE STATUSVARIABLEN (Definitionen) ---
bool serverStarted = false;
bool startApMode = false;
LogLevel activeLogLevel = LOG_DEBUG;
std::vector<String> startupLogBuffer;

// PIN PLATZHALTER
const int PIN_LIGHT_IN = D5;
const int PIN_FARLIGHT_IN = D6;
const int PIN_BRAKE_IN = D7;
const int PIN_BLINK_LEFT = D4;
const int PIN_BLINK_RIGHT = D3;

// --- Konfigurations-Structs (Definitionen) ---

// Beispiel: Die korrigierte RpmConfig Initialisierung (Aktion 35)
RpmConfig rpmConfig = {2, 10000}; // {pulses_per_revolution, max_rpm}
IgnitionTimingConfig timingConfig = {60, 0.0f};
IgnitionCoilConfig coilConfig = {0.2f, 2.0f, 4.0f, 8.0f, 3.5f};
struct SpeedConfig speedConfig = {14, 1985};

// --- Logger Definitionen (Aktion 38) ---
// LogLevel activeLogLevel wird in der Konfigurationsfunktion überschrieben
// LogLevel activeLogLevel = LOG_DEBUG; 

// --- Weitere Konfigurationen (Initialisierung der structs) ---
// HINWEIS: Prüfen Sie Ihre CoreDefs.h auf die exakte Struktur. 
// Ich nehme an, dass die structs IgnitionTimingConfig und IgnitionCoilConfig fehlen.
// Diese müssen Sie entweder in CoreDefs.h definieren oder die Initialisierung entfernen.

// --- ZENTRALE DATENSTRUKTUREN ---
struct IgnitionState
{
    volatile unsigned long last_pulse_micros;
    volatile unsigned long period_micros;
    volatile int rpm;
    volatile unsigned long advance_micros;
    volatile bool pulse_detected;
} ignition = {0, 0, 0, 0, false};

struct SpeedState
{
    volatile unsigned long last_pulse_micros;
    volatile unsigned long period_micros;
    volatile float speed_kmh_raw;
} speedState = {0, 0, 0.0f};

struct SensorState
{
    volatile int tps_raw;
    volatile int geschwindigkeit_kmh;
    volatile bool licht_ein_state;
    volatile bool fernlicht_ein_state;
    volatile bool bremslicht_state;
    volatile bool blinker_links_state;
    volatile bool blinker_rechts_state;
    volatile float batteriespannung_v;
    volatile float temperatur_zylinderkopf_c;
    volatile float temperatur_luftfilterkasten_c;
    volatile float temperatur_umgebung_c;
    volatile float luftfeuchtigkeit_umgebung_pct;
} sensor = {0, 0, false, false, false, false, false, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

//IgnitionMap2D ignitionMap2D; // NEU: Definition der 2D-Map
class BaseSensor
{
public:
    virtual ~BaseSensor() {}
    virtual float readValue() = 0;
    virtual unsigned long getMinInterval() = 0;
};
class TPS_Sensor : public BaseSensor
{
private:
    const int _pin;
    const unsigned long _interval;

public:
    TPS_Sensor(int pin, unsigned long interval_ms) : _pin(pin), _interval(interval_ms) {}
    unsigned long getMinInterval() override { return _interval; }
    float readValue() override { return (float)analogRead(_pin); }
};
class DigitalInput_Sensor : public BaseSensor
{
private:
    const int _pin;
    const unsigned long _interval;
    const bool _pullup;

public:
    DigitalInput_Sensor(int pin, unsigned long interval_ms, bool pullup = true) : _pin(pin), _interval(interval_ms), _pullup(pullup) { pinMode(_pin, _pullup ? INPUT_PULLUP : INPUT); }
    unsigned long getMinInterval() override { return _interval; }
    float readValue() override { return (digitalRead(_pin) == (_pullup ? LOW : HIGH)) ? 1.0f : 0.0f; }
};
class Static_Sensor : public BaseSensor
{
private:
    const float _value;
    const unsigned long _interval;

public:
    Static_Sensor(float value, unsigned long interval_ms) : _value(value), _interval(interval_ms) {}
    unsigned long getMinInterval() override { return _interval; }
    float readValue() override { return _value; }
};
class CalibratedAnalog_Sensor : public BaseSensor
{
private:
    const float _adc_ref;
    const float _divider_ratio;
    const unsigned long _interval;

public:
    CalibratedAnalog_Sensor(float adc_ref, float divider_ratio, unsigned long interval_ms) : _adc_ref(adc_ref), _divider_ratio(divider_ratio), _interval(interval_ms) {}
    unsigned long getMinInterval() override { return _interval; }
    float readRawADC() { return 350.0f; }
    float readValue() override
    {
        float v_adc = readRawADC() * (_adc_ref / 1023.0f);
        return v_adc * _divider_ratio;
    }
};
class TempSensorSim : public BaseSensor
{
private:
    const unsigned long _interval;

public:
    TempSensorSim(unsigned long interval_ms) : _interval(interval_ms) {}
    unsigned long getMinInterval() override { return _interval; }
    float readValue() override { return 25.0f + (sin((float)millis() / 5000.0f) * 5.0f); }
};
class I2C_Expander
{
private:
    uint8_t _address;
    const unsigned long _interval;

public:
    unsigned long last_update_ms = 0;
    I2C_Expander(uint8_t address, unsigned long interval_ms) : _address(address), _interval(interval_ms) { LOG(LOG_INFO, "Expander 0x%X initialisiert.", _address); }
    unsigned long getMinInterval() const { return _interval; }
    uint16_t readPort() { return 0b1010101010101010; }
    void execute()
    {
        uint16_t port_value = readPort();
        if (_address == 0x20)
        {
            noInterrupts();
            sensor.blinker_links_state = (port_value & 0x0001);
            sensor.blinker_rechts_state = (port_value & 0x0002);
            interrupts();
        }
        LOG(LOG_DEBUG, "Expander 0x%X I/O gelesen. (Port: 0x%X)", _address, port_value);
    }
};

struct SensorTask
{
    std::unique_ptr<BaseSensor> sensor_ptr;
    unsigned long last_update_ms = 0;
    SensorID target_id;
};
std::vector<SensorTask> allSensorTasks;
static int current_task_index = 0;
std::vector<std::unique_ptr<I2C_Expander>> allExpanderTasks;

// --- STATISTIK & RESSOURCEN ---
const int STATS_WINDOW_SIZE = 100;
const unsigned long MAX_LOOP_DURATION_US = 10000;
const int MAX_LOG_LINES = 100;
const unsigned long MIN_HEAP_WARNING = 5000;

volatile unsigned long loop_durations_buffer[STATS_WINDOW_SIZE];
volatile int stats_index = 0;
volatile unsigned long max_loop_duration_us = 0;
volatile unsigned long min_loop_duration_us = 999999;
volatile unsigned long avg_loop_duration_us = 0;
volatile unsigned long last_critical_max_duration_us = 0;
volatile unsigned long last_critical_timestamp_ms = 0;
volatile bool is_critical_latency_active = false;

// --- Zeitbasiertes Web-Handling ---
unsigned long lastWebHandle = 0;
unsigned long lastOtaHandle = 0;
unsigned long lastHeapCheck = 0;
const unsigned long HEAP_CHECK_INTERVAL = 5000;

volatile unsigned long server_start_delay_ms = 30000;
volatile unsigned long boot_time_ms = 0;
static int calc_counter = 0;
const int CALC_FREQUENCY = 100;

// --- NOTFALL-FALLBACK KENNFIELD (PROGMEM) ---
// NEU: Im 2D-Achsen-Format (TPS-Achse | RPM-Achse | Datenmatrix)
const char *FALLBACK_CURVE_JSON = R"({"ssid": "", "password": "", "ap_ssid": "esp-aktor", "ap_password": "MeinSicheresPasswort", "rpm_pulses": 1, "rpm_type": true, "timing_offset": 60, "timing_tdc": 0.0, "primary_resistance_ohm": 0.2, "external_resistance_ohm": 2.0, "primary_inductance_mH": 4.0, "target_current_A": 8.0, "fixed_dwell_ms": 3.5, "log_level": 0, 
"sprocket_teeth": 14, "wheel_circumference_mm": 1985, 
"map2d": {
    "rpm_axis": [0, 2000, 4000, 8000],
    "tps_axis": [0, 256, 512, 1023],
    "angle_data": [
        [8, 10, 15, 12], 
        [10, 15, 20, 18],
        [12, 20, 25, 22],
        [15, 22, 28, 25]
    ]
}})";

// Ticker-Instanzen
Ticker advanceTicker;
Ticker debugTicker;
Ticker statsTicker;

// Globale Variablen für STA und AP Konfiguration
String wifiSsid = "";
String wifiPassword = "";
String apSsid = "esp-aktor";
String apPassword = "MeinSicheresPasswort";

// Instanz des Webservers (extern in WebServer.h deklariert)
ESP8266WebServer server(80);

// =========================================================
// 2. FORWARD-DEKLARATIONEN (Implementierung folgt hier)
// =========================================================

// Deklaration der kritischen Getter
unsigned long getIgnitionPeriodSafe();
int getIgnitionRpmSafe();
unsigned long getAdvanceMicrosSafe();
float getSpeedStateSafe();

// Helfer (Implementierung folgt)
void checkHeapHealth();
int calculateBilinearAngle(int rpm, int tps);
float calculateDwellTimeMs();
void updateIgnitionParameters();
void updateSensorState(SensorID id, float value);
void updateSensorsRoundRobin();
void updateSlowIOControllers();
void calculateLoopStatistics();
void printLoopStatus();
bool loadDataFromJson(const char *jsonString); // Load-Funktion

void updateSpeedCalibrationFactor();

// =========================================================
// 3. FUNKTIONSDEFINITIONEN (Kernlogik & Helper-Funktionen)
// =========================================================

// Definition der kritischen Getter (in CoreDefs.h extern deklariert)
unsigned long getIgnitionPeriodSafe()
{
    noInterrupts();
    unsigned long period = ignition.period_micros;
    interrupts();
    return period;
}
int getIgnitionRpmSafe()
{
    noInterrupts();
    int rpm = ignition.rpm;
    interrupts();
    return rpm;
}
unsigned long getAdvanceMicrosSafe()
{
    noInterrupts();
    unsigned long advance = ignition.advance_micros;
    interrupts();
    return advance;
}
bool isAuthorized() { return server.authenticate(HTTP_USERNAME, apPassword.c_str()); }
float getSpeedStateSafe()
{
    noInterrupts();
    unsigned long period = speedState.period_micros;
    float calFactor = 1.0f;
    interrupts();

    if (period > 1000000UL)
        return 0.0f;
    if (period == 0 || calFactor == 0.0f)
        return 0.0f;
    return calFactor / (float)period;
}

// --- SICHERE GETTER FÜR SENSORSTATE ---
float getSensorTpsRawSafe()
{
    noInterrupts();
    float val = sensor.tps_raw;
    interrupts();
    return val;
}
float getSensorBattVSafe()
{
    noInterrupts();
    float val = sensor.batteriespannung_v;
    interrupts();
    return val;
}
float getSensorTempKopfSafe()
{
    noInterrupts();
    float val = sensor.temperatur_zylinderkopf_c;
    interrupts();
    return val;
}
float getSensorSpeedSafe()
{
    noInterrupts();
    float val = sensor.geschwindigkeit_kmh;
    interrupts();
    return val;
}
// ------------------------------------

// In CoreDefs.h extern deklariert
int pinNameToCode(String pinName)
{
    pinName.toUpperCase();
    if (pinName.length() != 2 || pinName[0] != 'D')
        return -1;
    switch (pinName[1])
    {
    case '0':
        return D0;
    case '1':
        return D1;
    case '2':
        return D2;
    case '3':
        return D3;
    case '4':
        return D4;
    case '5':
        return D5;
    case '6':
        return D6;
    case '7':
        return D7;
    default:
        return -1;
    }
}
// In CoreDefs.h extern deklariert
void setPinState(int pinCode, bool targetState)
{
    if (pinCode == -1)
        return;
    if (pinCode == LED_PIN)
    {
        digitalWrite(pinCode, targetState ? LOW : HIGH);
    }
    else
    {
        digitalWrite(pinCode, targetState ? HIGH : LOW);
    }
    LOG(LOG_DEBUG, "Pin %d -> %s", pinCode, targetState ? "HIGH" : "LOW");
}

// WICHTIG: Helper-Funktion für die Interpolation entlang einer Achse (linear)
// Z = Z1 + (Z2 - Z1) * ((X - X1) / (X2 - X1))
float interpolateLinear(float x, float x1, float x2, float z1, float z2)
{
    if (x1 == x2)
        return z1; // Vermeidet Division durch Null (z.B. bei geclippten Werten)
    float factor = (x - x1) / (x2 - x1);
    return z1 + (z2 - z1) * factor;
}

// Implementierung der Bilinearen Interpolation
int calculateBilinearAngle(int rpm, int tps)
{
    // 1. Thread-sichere Kopie der Achsen und Daten
    std::vector<int> rpm_axis;                // X-Achse
    std::vector<int> tps_axis;                // Y-Achse
    std::vector<std::vector<int>> angle_data; // Z-Daten [tps][rpm]

    noInterrupts();
    rpm_axis = ignitionMap2D.rpm_axis;
    tps_axis = ignitionMap2D.tps_axis;
    angle_data = ignitionMap2D.angle_data;
    interrupts();

    size_t num_rpm = rpm_axis.size();
    size_t num_tps = tps_axis.size();

    // 2. Failsafe: Minimale Map-Größe prüfen
    if (num_rpm < 2 || num_tps < 2)
    {
        return 10;
    }

    // 3. Clamping der Input-Werte
    float rpm_clamped = (float)rpm;
    if (rpm_clamped < rpm_axis.front())
        rpm_clamped = rpm_axis.front();
    if (rpm_clamped > rpm_axis.back())
        rpm_clamped = rpm_axis.back();

    float tps_clamped = (float)tps;
    if (tps_clamped < tps_axis.front())
        tps_clamped = tps_axis.front();
    if (tps_clamped > tps_axis.back())
        tps_clamped = tps_axis.back();

    // 4. Index-Suche: Die vier umrahmenden Punkte (Q11, Q21, Q12, Q22) finden

    // Finde den Index des unteren Nachbarn auf der RPM-Achse (X1)
    size_t rpm_index1 = 0;
    for (size_t i = 0; i < num_rpm - 1; ++i)
    {
        if (rpm_axis[i + 1] > rpm_clamped)
        {
            rpm_index1 = i;
            break;
        }
        rpm_index1 = i + 1;
    }
    size_t rpm_index2 = std::min(rpm_index1 + 1, num_rpm - 1);

    // Finde den Index des unteren Nachbarn auf der TPS-Achse (Y1)
    size_t tps_index1 = 0;
    for (size_t j = 0; j < num_tps - 1; ++j)
    {
        if (tps_axis[j + 1] > tps_clamped)
        {
            tps_index1 = j;
            break;
        }
        tps_index1 = j + 1;
    }
    size_t tps_index2 = std::min(tps_index1 + 1, num_tps - 1);

    // 5. Hole die 4 Eckpunkte (Z11, Z21, Z12, Z22) und Achsenwerte (X1, X2, Y1, Y2)

    // Achsenwerte (Koordinaten des Gitters)
    float x1 = (float)rpm_axis[rpm_index1];
    float x2 = (float)rpm_axis[rpm_index2];
    float y1 = (float)tps_axis[tps_index1];
    float y2 = (float)tps_axis[tps_index2];

    // Z-Werte (Winkel an den Ecken)
    if (tps_index2 >= angle_data.size() || rpm_index2 >= angle_data[tps_index2].size())
        return 10;

    float z11 = (float)angle_data[tps_index1][rpm_index1]; // Q11 (X1, Y1)
    float z21 = (float)angle_data[tps_index1][rpm_index2]; // Q21 (X2, Y1)
    float z12 = (float)angle_data[tps_index2][rpm_index1]; // Q12 (X1, Y2)
    float z22 = (float)angle_data[tps_index2][rpm_index2]; // Q22 (X2, Y2)

    // 6. Bilineare Interpolation
    float z_a = interpolateLinear(rpm_clamped, x1, x2, z11, z21); // Interpolation entlang Y1
    float z_b = interpolateLinear(rpm_clamped, x1, x2, z12, z22); // Interpolation entlang Y2

    float final_angle = interpolateLinear(tps_clamped, y1, y2, z_a, z_b); // Finale Interpolation entlang X

    // 7. Ergebnis runden und zurückgeben
    return (int)std::round(final_angle);
}

// Implementierung der Dwell-Time Berechnung
float calculateDwellTimeMs()
{
    // 1. Hole die sicheren, aktuellen Werte
    float v_batt = getSensorBattVSafe(); // Echte Batteriespannung (V)

    float R_primary;
    float R_external;
    float L_mH;
    float I_target;
    float fixed_dwell;

    noInterrupts();
    R_primary = coilConfig.primary_resistance_ohm;
    R_external = coilConfig.external_resistance_ohm;
    L_mH = coilConfig.primary_inductance_mH;
    I_target = coilConfig.target_current_A;
    fixed_dwell = coilConfig.fixed_dwell_ms;
    interrupts();

    float R_total = R_primary + R_external; // Gesamtwiderstand in Ohm
    float L_Henry = L_mH / 1000.0f;         // Induktivität in Henry (für die Formel)

    // Failsafe/Boundary Checks
    if (v_batt < 8.0f || R_total < 0.1f || L_Henry < 0.0001f)
    {
        return fixed_dwell;
    }

    float I_max = v_batt / R_total;

    // Überprüfung, ob der Zielstrom I_target überhaupt erreicht werden kann
    if (I_target >= I_max)
    {
        return std::min(std::max(fixed_dwell, 4.0f), 8.0f);
    }

    // Berechnung der Dwell-Zeit in Sekunden (t_dwell = - (L/R) * ln(1 - (I_target * R) / V_batt))
    float exponent_term = 1.0f - (I_target * R_total) / v_batt;

    float t_dwell_seconds = -(L_Henry / R_total) * logf(exponent_term);

    float t_dwell_ms = t_dwell_seconds * 1000.0f;

    // Sicherheits-Clipping (1.0ms bis 8.0ms)
    return std::min(std::max(t_dwell_ms, 1.0f), 8.0f);
}

void IRAM_ATTR fireIgnitionOutput()
{
    digitalWrite(IGNITION_OUTPUT_PIN, HIGH);
    delayMicroseconds(50);
    digitalWrite(IGNITION_OUTPUT_PIN, LOW);
}

void IRAM_ATTR handleIgnitionPulse()
{
    unsigned long current_micros = micros();
    if (ignition.last_pulse_micros != 0)
    {
        ignition.period_micros = current_micros - ignition.last_pulse_micros;
        if (ignition.period_micros > 1000)
        {
            if (rpmConfig.pulses_per_revolution > 0)
            {
                ignition.rpm = (60000000UL / ignition.period_micros) / rpmConfig.pulses_per_revolution;
            }
            else
            {
                ignition.rpm = 0;
            }
        }
        unsigned long required_advance = ignition.advance_micros;
        if (required_advance > 100 && required_advance < ignition.period_micros)
        {
            timer1_attachInterrupt(fireIgnitionOutput);
            timer1_enable(TIM_DIV1, TIM_EDGE, TIM_SINGLE);
            timer1_write(required_advance * 5);
        }
    }
    ignition.last_pulse_micros = current_micros;
    ignition.pulse_detected = true;
}

void IRAM_ATTR handleSpeedPulse()
{
    unsigned long current_micros = micros();
    if (speedState.last_pulse_micros != 0)
    {
        speedState.period_micros = current_micros - speedState.last_pulse_micros;
    }
    speedState.last_pulse_micros = current_micros;
}

void updateSensorState(SensorID id, float value)
{
    noInterrupts();
    switch (id)
    {
    case SID_TPS_RAW:
        sensor.tps_raw = (int)std::round(value);
        sensor.bremslicht_state = (sensor.tps_raw > 500);
        break;
    case SID_SPEED:
        sensor.geschwindigkeit_kmh = (int)std::round(value);
        break;
    case SID_BATT_V:
        sensor.batteriespannung_v = value;
        break;
    case SID_TEMP_KOPF:
        sensor.temperatur_zylinderkopf_c = value;
        break;
    case SID_TEMP_LUFT:
        sensor.temperatur_luftfilterkasten_c = value;
        break;
    case SID_TEMP_UMG:
        sensor.temperatur_umgebung_c = value;
        break;
    case SID_FEUCHTIGKEIT:
        sensor.luftfeuchtigkeit_umgebung_pct = value;
        break;
    case SID_LIGHT:
        sensor.licht_ein_state = (value > 0.5f);
        break;
    case SID_FARLIGHT:
        sensor.fernlicht_ein_state = (value > 0.5f);
        break;
    case SID_BLINK_L:
        sensor.blinker_links_state = (value > 0.5f);
        break;
    case SID_BLINK_R:
        sensor.blinker_rechts_state = (value > 0.5f);
        break;
    default:
        break;
    }
    interrupts();
}

void updateSensorsRoundRobin()
{
    if (allSensorTasks.empty())
        return;
    unsigned long current_time = millis();
    SensorTask &currentTask = allSensorTasks[current_task_index];

    if ((current_time - currentTask.last_update_ms) >= currentTask.sensor_ptr->getMinInterval())
    {
        float value = currentTask.sensor_ptr->readValue();
        updateSensorState(currentTask.target_id, value);
        currentTask.last_update_ms = current_time;
    }
    current_task_index = (current_task_index + 1) % allSensorTasks.size();
}

void updateSlowIOControllers()
{
    if (allExpanderTasks.empty())
        return;
    unsigned long current_time = millis();
    for (auto &expander : allExpanderTasks)
    {
        if ((current_time - expander->last_update_ms) >= expander->getMinInterval())
        {
            expander->execute();
            expander->last_update_ms = current_time;
        }
    }
}

void updateIgnitionParameters()
{
    updateSensorsRoundRobin();
    updateSlowIOControllers();

    int rpm_copy = getIgnitionRpmSafe();
    unsigned long period = getIgnitionPeriodSafe();

    int tps_value;
    noInterrupts();
    tps_value = sensor.tps_raw;
    interrupts();

    int advance_angle = calculateBilinearAngle(rpm_copy, tps_value);

    // Dynamische DWELL-Berechnung
    float Dwell_ms = calculateDwellTimeMs();
    unsigned long Dwell_micros = (unsigned long)(Dwell_ms * 1000.0f);

    if (period > 1000)
    {
        float time_per_degree_micros = (float)period / 360.0f;
        unsigned long advance_time_micros = (unsigned long)(advance_angle * time_per_degree_micros);
        unsigned long total_advance_micros = advance_time_micros + Dwell_micros;

        noInterrupts();
        ignition.advance_micros = total_advance_micros;
        interrupts();
    }
    else
    {
        noInterrupts();
        ignition.advance_micros = 0;
        interrupts();
    }
}

// =========================================================
// 4. STATISTIK & LOGGING-LOGIK
// =========================================================

// logCriticalEvent ist jetzt in AppLogger.cpp implementiert

void calculateLoopStatistics()
{
    if (++calc_counter < CALC_FREQUENCY)
        return;
    calc_counter = 0;

    unsigned long sum = 0;
    unsigned long current_max = 0;
    unsigned long current_min = 999999;

    for (int i = 0; i < STATS_WINDOW_SIZE; i++)
    {
        unsigned long value = loop_durations_buffer[i];
        sum += value;
        current_max = std::max(current_max, value);
        if (value > 0)
            current_min = std::min(current_min, value);
    }

    avg_loop_duration_us = sum / STATS_WINDOW_SIZE;
    max_loop_duration_us = current_max;
    min_loop_duration_us = current_min;
}

void checkHeapHealth()
{
    uint32_t freeHeap = ESP.getFreeHeap();
    if (freeHeap < MIN_HEAP_WARNING)
    {
        LOG(LOG_WARN, "⚠️ Heap niedrig: %d Bytes frei!", freeHeap);
    }
    if (freeHeap < 2000)
    {
        LOG(LOG_ERROR, "🚨 KRITISCH: Nur noch %d Bytes Heap! Neustart empfohlen.", freeHeap);
    }
}

void printLoopStatus()
{
    calculateLoopStatistics();

    int rpm = getIgnitionRpmSafe();
    unsigned long advance = getAdvanceMicrosSafe();

    int tps_raw;
    float temp_kopf, batt_v;
    noInterrupts();
    tps_raw = sensor.tps_raw;
    temp_kopf = sensor.temperatur_zylinderkopf_c;
    batt_v = sensor.batteriespannung_v;
    interrupts();

    LOG(LOG_DEBUG, "RPM:%d | TPS:%d | ADV:%luµs | Dwell:%luµs | L(MAX:%lu/AVG:%lu)µs | CHT:%.1f°C | BAT:%.1fV | Heap:%d",
        rpm, tps_raw, advance, (unsigned long)(calculateDwellTimeMs() * 1000.0f),
        max_loop_duration_us, avg_loop_duration_us,
        temp_kopf, batt_v, ESP.getFreeHeap());

    if (max_loop_duration_us > MAX_LOOP_DURATION_US && !is_critical_latency_active)
    {
        is_critical_latency_active = true;
        last_critical_max_duration_us = max_loop_duration_us;
        last_critical_timestamp_ms = millis();
        LOG(LOG_WARN, "Latenz kritisch! Max: %lu µs", max_loop_duration_us);
    }
    else if (max_loop_duration_us <= MAX_LOOP_DURATION_US && is_critical_latency_active)
    {
        is_critical_latency_active = false;
    }
}

// =========================================================
// 5. KONFIGURATION UND HANDLER (Load/Save Logik)
// =========================================================

void updateSpeedCalibrationFactor()
{
    noInterrupts();
    // Hier müsste die Berechnung des Kalibrierungsfaktors basierend auf speedConfig.sprocket_teeth und wheel_circumference_mm erfolgen
    interrupts();
}

bool loadDataFromJson(const char *jsonString)
{
    // NEU: DynamicJsonDocument auf dem Heap allokieren, um den kritischen Stack zu entlasten
    const size_t CAPACITY = 2048;
    std::unique_ptr<DynamicJsonDocument> doc(new DynamicJsonDocument(CAPACITY));

    if (!doc)
    {
        LOG(LOG_ERROR, "Speicherallokation für JSON-Dokument fehlgeschlagen.");
        return false;
    }

    // Deserialisierung des JSON-Strings in das neue Heap-Dokument
    DeserializationError error = deserializeJson(*doc, jsonString);

    if (error)
    {
        LOG(LOG_ERROR, "JSON Fehlercode: %s", error.c_str());
        return false;
    }

    // --- LÄDT KONFIGURATION (ALLE ZUGRIFFE ANPASSEN) ---
    // Alle doc[...] Zugriffe werden zu (*doc)[...]

    wifiSsid = (*doc)[F("ssid")].as<String>();
    wifiPassword = (*doc)[F("password")] | "";
    apSsid = (*doc)[F("ap_ssid")] | "esp-aktor";
    apPassword = (*doc)[F("ap_password")] | "MeinSicheresPasswort";

    int levelInt = (*doc)[F("log_level")] | LOG_DEBUG;
    activeLogLevel = (LogLevel)levelInt;

    rpmConfig.pulses_per_revolution = (*doc)[F("rpm_pulses")] | 1;
    timingConfig.trigger_offset_deg = (*doc)[F("timing_offset")] | 60;
    coilConfig.primary_resistance_ohm = (*doc)[F("primary_resistance_ohm")] | 0.2f;
    coilConfig.external_resistance_ohm = (*doc)[F("external_resistance_ohm")] | 2.0f;
    coilConfig.fixed_dwell_ms = (*doc)[F("fixed_dwell_ms")] | 3.5f;

    // LÄDT GESCHWINDIGKEITSPARAMETER
    speedConfig.sprocket_teeth = (*doc)[F("sprocket_teeth")] | 14;
    speedConfig.wheel_circumference_mm = (*doc)[F("wheel_circumference_mm")] | 1985;

    // DEBUG AUSGABE...
    LOG(LOG_DEBUG, "--- GELADENE KONFIGURATION ---");
    LOG(LOG_DEBUG, "STA SSID: %s (Länge: %d)", wifiSsid.c_str(), wifiSsid.length());
    LOG(LOG_DEBUG, "Dwell: %.1fms | Teeth: %d | Circumference: %dmm", coilConfig.fixed_dwell_ms, speedConfig.sprocket_teeth, speedConfig.wheel_circumference_mm);

    // ----------------------------------------------------------------
    // MAP LADEN UND VALIDIERUNG (2D-Achsen-Format)
    // ----------------------------------------------------------------

    JsonObject map2d = (*doc)[F("map2d")];
    if (map2d.isNull() || !map2d.containsKey(F("rpm_axis")) || !map2d.containsKey(F("tps_axis")) || !map2d.containsKey(F("angle_data")))
    {
        LOG(LOG_ERROR, "2D Map Format (map2d) fehlt oder ist ungültig.");
        return false;
    }

    JsonArray rpmAxisArray = map2d[F("rpm_axis")];
    JsonArray tpsAxisArray = map2d[F("tps_axis")];
    JsonArray angleDataArray = map2d[F("angle_data")];

    size_t rpm_size = rpmAxisArray.size();
    size_t tps_size = tpsAxisArray.size();

    if (rpm_size < 2 || tps_size < 2 || angleDataArray.size() != tps_size)
    {
        LOG(LOG_ERROR, "2D Map Achsen sind zu klein oder Map-Größen stimmen nicht überein (%d x %d vs %d Rows).",
            rpm_size, tps_size, angleDataArray.size());
        return false;
    }

    // 1. Achsen laden und validieren (RPM und TPS müssen aufsteigend sein)
    ignitionMap2D.rpm_axis.clear();
    for (int r : rpmAxisArray)
    {
        if (r < 0 || (!ignitionMap2D.rpm_axis.empty() && r <= ignitionMap2D.rpm_axis.back()))
        {
            LOG(LOG_ERROR, "RPM-Achse ist ungültig oder nicht aufsteigend.");
            return false;
        }
        ignitionMap2D.rpm_axis.push_back(r);
    }

    ignitionMap2D.tps_axis.clear();
    for (int t : tpsAxisArray)
    {
        if (t < 0 || t > 1023 || (!ignitionMap2D.tps_axis.empty() && t <= ignitionMap2D.tps_axis.back()))
        {
            LOG(LOG_ERROR, "TPS-Achse ist ungültig oder nicht aufsteigend (Max 1023).");
            return false;
        }
        ignitionMap2D.tps_axis.push_back(t);
    }

    // 2. Datenmatrix laden und validieren
    ignitionMap2D.angle_data.clear();
    size_t tps_row_index = 0;

    for (JsonArray tpsRow : angleDataArray)
    {
        if (tpsRow.size() != rpm_size)
        {
            LOG(LOG_ERROR, "Zeile %d der Angle-Daten hat falsche Spaltengröße (%d, erwartet %d).",
                tps_row_index, tpsRow.size(), rpm_size);
            return false;
        }

        std::vector<int> row;
        for (int angle : tpsRow)
        {
            if (angle < -20 || angle > 30)
            {
                LOG(LOG_ERROR, "Zündwinkel in Zeile %d ist außerhalb des zulässigen Bereichs (-20 bis 30).", tps_row_index);
                return false;
            }
            row.push_back(angle);
        }
        ignitionMap2D.angle_data.push_back(row);
        tps_row_index++;
    }

    LOG(LOG_INFO, "2D-Kennfeld geladen: %d x %d Punkte.", rpm_size, tps_size);

    updateSpeedCalibrationFactor();
    // Der Heap-Speicher (*doc) wird freigegeben, wenn die Funktion endet (dank unique_ptr)
    return true;
}

// Hier muss saveConfig() als komplette Definition stehen!
bool saveConfig()
{
    StaticJsonDocument<2048> doc;

    // SPEICHERN ALLER KONFIGURATIONEN
    doc[F("ssid")] = wifiSsid;
    doc[F("password")] = wifiPassword;
    doc[F("ap_ssid")] = apSsid;
    doc[F("ap_password")] = apPassword;
    doc[F("log_level")] = (int)activeLogLevel;
    doc[F("rpm_pulses")] = rpmConfig.pulses_per_revolution;
    // ... (restliche Config-Parameter speichern) ...
    doc[F("sprocket_teeth")] = speedConfig.sprocket_teeth;
    doc[F("wheel_circumference_mm")] = speedConfig.wheel_circumference_mm;

    // MAP IM 2D-ACHSEN-FORMAT SPEICHERN (Hier ist der lange Teil)
    JsonObject map2d = doc.createNestedObject(F("map2d"));
    // ... (Logik zur Speicherung von rpm_axis, tps_axis und angle_data) ...

    File configFile = LittleFS.open(F("/config.json"), "w");
    if (!configFile)
        return false;
    bool success = (serializeJson(doc, configFile) > 0);
    configFile.close();
    if (success)
        LOG(LOG_INFO, "Config gespeichert.");
    return success;
}

bool loadConfig()
{
    // ------------------------------------------------------------------------
    // SCHRITT 1: Fallback-Werte setzen (FÜR ERSTEN BOOT ODER FEHLERHAFTEN START)
    // ------------------------------------------------------------------------

    // Fallback-Werte für WLAN (Hier Ihre Daten für den STA-Modus Test eintragen!)
    wifiSsid = F("Ihre_Heim_SSID_hier");        // <--- BITTE HIER ERSETZEN
    wifiPassword = F("Ihr_WLAN_Passwort_hier"); // <--- BITTE HIER ERSETZEN

    // Fallback für AP-Modus
    apSsid = F("esp-aktor");
    apPassword = F("MeinSicheresPasswort");

    // Fallback für Core-Einstellungen
    activeLogLevel = LOG_INFO;
    rpmConfig.pulses_per_revolution = 2; // Zündimpulse pro Umdrehung (Default 2)
    rpmConfig.max_rpm = 10000;

    // Fallback für Geschwindigkeitskalibrierung
    speedConfig.sprocket_teeth = 15;
    speedConfig.wheel_circumference_mm = 1890;

    // Fallback für Zündwinkel (Statische 10° im gesamten Feld)
    for (int i = 0; i < MAP_SIZE; ++i)
    {
        for (int j = 0; j < MAP_SIZE; ++j)
        {
            angle_data[i][j] = 10;
        }
    }

    // Fallback für Achsen (Beispiel-Achsen)
    // RPM: 0, 1000, 2000, 4000, 8000, 12000, 16000, 20000
    rpm_axis[0] = 0;
    rpm_axis[1] = 1000;
    rpm_axis[2] = 2000;
    rpm_axis[3] = 4000;
    rpm_axis[4] = 8000;
    rpm_axis[5] = 12000;
    rpm_axis[6] = 16000;
    rpm_axis[7] = 20000;

    // TPS: 0%, 10%, 20%, 30%, 50%, 70%, 90%, 100%
    tps_axis[0] = 0;
    tps_axis[1] = 10;
    tps_axis[2] = 20;
    tps_axis[3] = 30;
    tps_axis[4] = 50;
    tps_axis[5] = 70;
    tps_axis[6] = 90;
    tps_axis[7] = 100;

    // ------------------------------------------------------------------------
    // SCHRITT 2: Versuche, die Konfigurationsdatei vom LittleFS zu laden
    // ------------------------------------------------------------------------
    File configFile = LittleFS.open(F("/config.json"), "r");

    if (!configFile)
    {
        // Kritisch: Wenn keine Datei existiert, speichern wir die neuen Fallbacks.
        LOG(LOG_WARN, "Konfigurationsdatei nicht gefunden. Speichere neue Fallbacks.");
        if (saveConfig())
        {
            LOG(LOG_INFO, "Fallback-Konfiguration erfolgreich gespeichert.");
            return true;
        }
        else
        {
            LOG(LOG_ERROR, "Fallback-Konfiguration konnte nicht gespeichert werden.");
            return false;
        }
    }

    // Reserviere Speicher für das JSON-Dokument
    // Berechnung: Base + (2 * MAP_SIZE * sizeof(int)) + (MAP_SIZE*MAP_SIZE * sizeof(int)) + String Overheads
    StaticJsonDocument<2048> doc;

    DeserializationError error = deserializeJson(doc, configFile);
    configFile.close();

    if (error)
    {
        LOG(LOG_ERROR, "Failed to parse config file: %s. Verwende Fallbacks.", error.c_str());
        // Trotz Parsen-Fehler verwenden wir die Fallbacks, speichern diese aber nicht erneut.
        return true;
    }

    // ------------------------------------------------------------------------
    // SCHRITT 3: Konfiguration aus JSON übernehmen
    // ------------------------------------------------------------------------

    // WLAN
    if (doc.containsKey(F("ssid")))
        wifiSsid = doc[F("ssid")].as<String>();
    if (doc.containsKey(F("password")))
        wifiPassword = doc[F("password")].as<String>();
    if (doc.containsKey(F("ap_ssid")))
        apSsid = doc[F("ap_ssid")].as<String>();
    if (doc.containsKey(F("ap_password")))
        apPassword = doc[F("ap_password")].as<String>();

    // Logging
    if (doc.containsKey(F("log_level")))
        activeLogLevel = (LogLevel)doc[F("log_level")].as<int>();

    // RPM
    if (doc.containsKey(F("rpm_pulses")))
        rpmConfig.pulses_per_revolution = doc[F("rpm_pulses")].as<int>();
    if (doc.containsKey(F("max_rpm")))
        rpmConfig.max_rpm = doc[F("max_rpm")].as<int>();

    // Speed
    if (doc.containsKey(F("sprocket_teeth")))
        speedConfig.sprocket_teeth = doc[F("sprocket_teeth")].as<int>();
    if (doc.containsKey(F("wheel_circumference_mm")))
        speedConfig.wheel_circumference_mm = doc[F("wheel_circumference_mm")].as<int>();

    // Zündwinkel-Map
    if (doc.containsKey(F("map2d")))
    {
        JsonObject map2d = doc[F("map2d")];

        // RPM Axis
        if (map2d.containsKey(F("rpm_axis")) && map2d[F("rpm_axis")].is<JsonArray>())
        {
            JsonArray rpmArray = map2d[F("rpm_axis")].as<JsonArray>();
            for (size_t i = 0; i < rpmArray.size() && i < MAP_SIZE; ++i)
            {
                rpm_axis[i] = rpmArray[i].as<int>();
            }
        }

        // TPS Axis
        if (map2d.containsKey(F("tps_axis")) && map2d[F("tps_axis")].is<JsonArray>())
        {
            JsonArray tpsArray = map2d[F("tps_axis")].as<JsonArray>();
            for (size_t i = 0; i < tpsArray.size() && i < MAP_SIZE; ++i)
            {
                tps_axis[i] = tpsArray[i].as<int>();
            }
        }

        // Angle Data
        if (map2d.containsKey(F("angle_data")) && map2d[F("angle_data")].is<JsonArray>())
        {
            JsonArray dataArray = map2d[F("angle_data")].as<JsonArray>();
            for (size_t i = 0; i < dataArray.size() && i < MAP_SIZE; ++i)
            {
                JsonArray row = dataArray[i].as<JsonArray>();
                for (size_t j = 0; j < row.size() && j < MAP_SIZE; ++j)
                {
                    angle_data[i][j] = row[j].as<int>();
                }
            }
        }
    }

    LOG(LOG_INFO, "Konfiguration aus Datei geladen. Heap: %d Bytes", ESP.getFreeHeap());
    return true;
}

void setup()
{
    Serial.begin(115200);
    delay(100);

    Serial.println(F("\n--- SYSTEM START: Logger Test (115200 Baud) ---"));
    LOG(LOG_INFO, "Serielle Kommunikation initialisiert.");

    // 1. Zündungs-Core initialisieren
    pinMode(IGNITION_INPUT_PIN, INPUT_PULLUP);
    pinMode(IGNITION_OUTPUT_PIN, OUTPUT);
    pinMode(SPEED_INPUT_PIN, INPUT_PULLUP);
    digitalWrite(IGNITION_OUTPUT_PIN, LOW);

    // Setze den Timer und die Interrupts
    timer1_disable();
    attachInterrupt(digitalPinToInterrupt(IGNITION_INPUT_PIN), handleIgnitionPulse, RISING);
    attachInterrupt(digitalPinToInterrupt(SPEED_INPUT_PIN), handleSpeedPulse, RISING);

    // 2. Initialisiere die schnellen, polymorphen Sensor-Tasks
    allSensorTasks.push_back({std::make_unique<TPS_Sensor>(ADC_PIN, 50), 0, SID_TPS_RAW});
    allSensorTasks.push_back({std::make_unique<Static_Sensor>(0.0f, 50), 0, SID_SPEED});
    allSensorTasks.push_back({std::make_unique<CalibratedAnalog_Sensor>(3.3f, 11.0f, 1000), 0, SID_BATT_V});
    allSensorTasks.push_back({std::make_unique<TempSensorSim>(1000), 0, SID_TEMP_KOPF});

    // 3. Konfiguration und WLAN
    if (!LittleFS.begin())
    {
        LOG(LOG_WARN, "LittleFS formatiert.");
        LittleFS.format();
    }

    if (!loadConfig())
    {
        LOG(LOG_ERROR, "Kritischer Konfigurationsfehler beim Boot.");
    }

    LOG(LOG_INFO, "WLAN Setup: Verbinde mit '%s' (PW-Länge: %d)", wifiSsid.c_str(), wifiSsid.length());

    WiFi.mode(WIFI_STA);
    WiFi.begin(wifiSsid.c_str(), wifiPassword.c_str());
    LOG(LOG_INFO, "Versuche, mit %s zu verbinden...", wifiSsid.c_str());

    unsigned long startTime = millis();
    const unsigned long TIMEOUT = 15000;
    while (WiFi.status() != WL_CONNECTED && (millis() - startTime < TIMEOUT))
    {
        delay(100);
        ESP.wdtFeed();
    }

    // --- Start des Haupt-Webserver-Setups ---
    setup_server_routes(); // Routen registrieren (KEIN server.begin()!)

    if (WiFi.status() == WL_CONNECTED)
    {
        // STA-Modus: Erfolg
        LOG(LOG_INFO, "✅ Verbunden! IP: %s", WiFi.localIP().toString().c_str());

        // HINWEIS: server.begin() und setup_ota() werden verzögert in loop() ausgeführt.

        advanceTicker.attach_ms(50, updateIgnitionParameters);
        debugTicker.attach(1, printLoopStatus);
        statsTicker.attach_ms(500, calculateLoopStatistics);

        LOG(LOG_INFO, "System bereit. Heap: %d Bytes", ESP.getFreeHeap());
    }
    else
    {
        // AP-Modus: Verbindung fehlgeschlagen
        LOG(LOG_WARN, "Verbindung fehlgeschlagen. Starte AP-Modus.");
        startApMode = true;

        WiFi.mode(WIFI_AP);

        // FIX 1: Explizite AP Konfiguration für stabile Netzwerkschicht
        WiFi.softAPConfig(apIP, gatewayIP, netMask);

        LOG(LOG_INFO, "WiFi Mode auf AP gesetzt. Starte SoftAP.");
        WiFi.softAP(apSsid.c_str(), apPassword.c_str());

        LOG(LOG_INFO, "SoftAP gestartet. Konfiguriere HTTP-Routen.");
        // Routen wurden bereits oben einmal registriert.

        // FIX 2: Heap Konsolidierung, bevor der Webserver in loop() startet
        yield();
        delay(1);

        LOG(LOG_INFO, "HTTP-Routen konfiguriert. Lösche Startup-Log-Buffer.");
        startupLogBuffer.clear();

        LOG(LOG_INFO, "AP IP: %s | Heap: %d Bytes",
            WiFi.softAPIP().toString().c_str(),
            ESP.getFreeHeap());
    }
}

// main.cpp (loop() Funktion)

void loop()
{
    ESP.wdtFeed();

    unsigned long now = millis();

    // 1. AP/STA Modus Handling
    if (startApMode)
    {
        // NEU: Verzögerter Webserver-Start
        if (!webserverRunning)
        {
            server.begin();
            webserverRunning = true;
            LOG(LOG_INFO, "WEB: Server jetzt in loop() gestartet.");
        }

        // WICHTIG: Explizites Setzen des AP-Modus und ungdedrosselter Aufruf
        WiFi.mode(WIFI_AP);
        server.handleClient();
        updateIgnitionParameters(); // Muss im AP-Mode manuell laufen

        // Loggen des Lebenszeichens (alle 2 Sekunden)
        if (now % 2000 < 50)
        {
            LOG(LOG_INFO, "AP-Loop aktiv und stabil.");
        }
    }
    else
    {
        // STA Mode
        // NEU: Verzögerter Webserver-Start (auch hier nötig)
        if (!webserverRunning)
        {
            server.begin();
            webserverRunning = true;
            LOG(LOG_INFO, "WEB: Server jetzt in loop() (STA) gestartet.");
            setup_ota(); // OTA-Setup hier triggern
        }

        // Gedrosselte Webserver-Behandlung
        if (now - lastWebHandle >= WEB_HANDLE_INTERVAL)
        {
            server.handleClient();
            lastWebHandle = now;
        }
        if (now - lastOtaHandle >= OTA_HANDLE_INTERVAL)
        {
            ArduinoOTA.handle();
            lastOtaHandle = now;
        }
        // updateIgnitionParameters() läuft hier im Ticker
    }

    // 2. Heap Check und Statistik-Aktualisierung (Zurückstellen, da es den Absturz auslöste)
    // Nur das Nötigste läuft, um Stabilität zu gewährleisten.

    // 5. Critical Logic: Triggert den Scheduler
    delay(0);
}