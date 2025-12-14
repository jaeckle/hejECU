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

// NEUE HEADER INKLUSIONEN
#include "CoreDefs.h"
#include "WebServer.h"

#pragma GCC optimize("O3")

// =========================================================
// 1. GLOBALE DEFS UND LOGGER
// =========================================================

// --- ANDERE KONFIGURATION (Konstanten HIER DEFINIERT, um Sichtbarkeit zu gewährleisten) ---
const int LED_PIN = D4;
const int BUTTON_PIN = D5;
const char *HTTP_USERNAME = "admin";

// Definition des Log-Makros (Forward-Deklaration in CoreDefs.h)
#define LOG(level, format, ...) app_log(level, __FILE__, __LINE__, __LINE__, format, ##__VA_ARGS__)

// --- ZUSÄTZLICHE IN-RAM LOGGING STRUKTUR FÜR STARTPHASE (Definition) ---
std::vector<String> startupLogBuffer;
const size_t MAX_STARTUP_LOGS = 50;

// --- WICHTIGE GLOBALE STATUSVARIABLEN (Definitionen) ---
bool serverStarted = false;
bool startApMode = false;
volatile LogLevel activeLogLevel = LOG_DEBUG;

// ZENTRALER LOGGER DEFINITION
void app_log(LogLevel level, const char *file, int line, int logLine, const char *format, ...)
{
    if (level < activeLogLevel)
        return;

    const char *levelStr = "UNKNOWN";
    switch (level)
    {
    case LOG_DEBUG:
        levelStr = "DEBUG";
        break;
    case LOG_INFO:
        levelStr = "INFO";
        break;
    case LOG_WARN:
        levelStr = "WARNUNG";
        break;
    case LOG_ERROR:
        levelStr = "FEHLER";
        break;
    }

    unsigned long timeMs = millis();
    char logBuffer[256];

    int len = snprintf(logBuffer, sizeof(logBuffer), "[%lu ms] [%s] (%s:%d) ",
                       timeMs, levelStr, file, logLine);

    va_list args;
    va_start(args, format);
    vsnprintf(logBuffer + len, sizeof(logBuffer) - len, format, args);
    va_end(args);

    Serial.println(logBuffer);

    if (!serverStarted && startupLogBuffer.size() < MAX_STARTUP_LOGS)
    {
        startupLogBuffer.push_back(String(logBuffer));
    }
}

// --- ZÜNDUNGSKONFIGURATION ---
const int IGNITION_INPUT_PIN = D1;
const int IGNITION_OUTPUT_PIN = D2;
const int ADC_PIN = A0;
const int SPEED_INPUT_PIN = D0;

// PIN PLATZHALTER
const int PIN_LIGHT_IN = D5;
const int PIN_FARLIGHT_IN = D6;
const int PIN_BRAKE_IN = D7;
const int PIN_BLINK_LEFT = D4;
const int PIN_BLINK_RIGHT = D3;

// --- Konfigurations-Structs (Definitionen) ---
struct RpmConfig rpmConfig = {1, true, IGNITION_INPUT_PIN};
struct IgnitionTimingConfig timingConfig = {60, 0.0f};
struct IgnitionCoilConfig coilConfig = {0.2f, 2.0f, 4.0f, 8.0f, 3.5f};

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

IgnitionMap2D ignitionMap2D; // NEU: Definition der 2D-Map

// --- SENSOR ABSTRAKTION UND VERWALTUNG (Klassen) ---
enum SensorID
{
    SID_TPS_RAW,
    SID_SPEED,
    SID_BATT_V,
    SID_TEMP_KOPF,
    SID_TEMP_LUFT,
    SID_TEMP_UMG,
    SID_FEUCHTIGKEIT,
    SID_LIGHT,
    SID_FARLIGHT,
    SID_BRAKE,
    SID_BLINK_L,
    SID_BLINK_R,
    SID_COUNT
};
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
const char *LOG_FILENAME = "/critical_log.txt";
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
const unsigned long WEB_HANDLE_INTERVAL = 200;
const unsigned long OTA_HANDLE_INTERVAL = 1000;
const unsigned long HEAP_CHECK_INTERVAL = 5000;

volatile unsigned long server_start_delay_ms = 30000;
volatile unsigned long boot_time_ms = 0;
static int calc_counter = 0;
const int CALC_FREQUENCY = 100;

// --- NOTFALL-FALLBACK KENNFIELD (PROGMEM) ---
// NEU: Im 2D-Achsen-Format (TPS-Achse | RPM-Achse | Datenmatrix)
const char *FALLBACK_CURVE_JSON = R"({"ssid": "", "password": "", "ap_ssid": "esp-aktor", "ap_password": "MeinSicheresPasswort", "rpm_pulses": 1, "rpm_type": true, "timing_offset": 60, "timing_tdc": 0.0, "primary_resistance_ohm": 0.2, "external_resistance_ohm": 2.0, "primary_inductance_mH": 4.0, "target_current_A": 8.0, "fixed_dwell_ms": 3.5, "log_level": 0, 
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

// TEMPORÄRE IMPLEMENTIERUNG: Lineare Interpolation (wird im nächsten Schritt ersetzt)
int calculateBilinearAngle(int rpm, int tps)
{
    // 1. Failsafe und Thread-Sicherheit
    if (ignitionMap2D.rpm_axis.empty())
        return 10; // Standardwinkel 10° bei leerer Map

    // NOTE: Wir verwenden hier rpm_axis als Ersatz für die frühere, sortierte flache Map.
    std::vector<int> rpmAxisCopy;
    noInterrupts();
    rpmAxisCopy = ignitionMap2D.rpm_axis;
    interrupts();

    // 2. BOUNDARY CHECKING (Clipping der RPM-Werte)

    int minRpm = rpmAxisCopy.front();
    int maxRpm = rpmAxisCopy.back();

    if (rpm <= minRpm)
        rpm = minRpm;
    if (rpm >= maxRpm)
        rpm = maxRpm;

    // 3. Finden der umrahmenden Punkte (Q_low und Q_high)

    int rpm_low_value = minRpm;
    int rpm_high_value = maxRpm;
    int angle_low = 10;
    int angle_high = 10;

    // Suche nach den nächstgelegenen Achsen-Werten (nur RPM)
    for (size_t i = 0; i < rpmAxisCopy.size(); ++i)
    {
        int r = rpmAxisCopy[i];

        if (r <= rpm)
        {
            rpm_low_value = r;
            // TEMPORÄRE FALLBACKS (Müssen im echten Bilinear-Code ersetzt werden):
            // Wir verwenden den ersten TPS-Row (Index 0) als Näherung für den Winkel.
            if (!ignitionMap2D.angle_data.empty() && !ignitionMap2D.angle_data[0].empty() && i < ignitionMap2D.angle_data[0].size())
            {
                angle_low = ignitionMap2D.angle_data[0][i];
            }
        }

        if (r >= rpm)
        {
            rpm_high_value = r;
            if (!ignitionMap2D.angle_data.empty() && !ignitionMap2D.angle_data[0].empty() && i < ignitionMap2D.angle_data[0].size())
            {
                angle_high = ignitionMap2D.angle_data[0][i];
            }
            break; // Erste obere Grenze gefunden
        }
    }

    // 4. LINEARE INTERPOLATION (entlang der RPM-Achse)

    float angle = 0.0f;

    if (rpm_low_value == rpm_high_value)
    {
        angle = angle_low;
    }
    else
    {
        float factor = (float)(rpm - rpm_low_value) / (float)(rpm_high_value - rpm_low_value);
        angle = angle_low + factor * (angle_high - angle_low);
    }

    return (int)round(angle);
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

    float Dwell_ms_calc;
    float fixed_dwell;
    noInterrupts();
    float R_total = coilConfig.primary_resistance_ohm + coilConfig.external_resistance_ohm;
    float L_mH = coilConfig.primary_inductance_mH;
    fixed_dwell = coilConfig.fixed_dwell_ms;
    interrupts();

    if (R_total > 0.1f)
    {
        Dwell_ms_calc = L_mH / R_total;
    }
    else
    {
        Dwell_ms_calc = fixed_dwell;
    }
    unsigned long Dwell_micros = (unsigned long)(std::max(Dwell_ms_calc, fixed_dwell) * 1000.0f);

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

void logCriticalEvent(unsigned long maxDuration, unsigned long timestamp, const char *status); // Deklaration beibehalten

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

    LOG(LOG_DEBUG, "RPM:%d | TPS:%d | ADV:%luµs | L(MAX:%lu/AVG:%lu)µs | CHT:%.1f°C | BAT:%.1fV | Heap:%d",
        rpm, tps_raw, advance,
        max_loop_duration_us, avg_loop_duration_us,
        temp_kopf, batt_v, ESP.getFreeHeap());

    if (max_loop_duration_us > MAX_LOOP_DURATION_US && !is_critical_latency_active)
    {
        is_critical_latency_active = true;
        last_critical_max_duration_us = max_loop_duration_us;
        last_critical_timestamp_ms = millis();
        // logCriticalEvent(last_critical_max_duration_us, last_critical_timestamp_ms, "CRITICAL_LATENCY"); // In WebServer.cpp definiert
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
    interrupts();
}

bool loadDataFromJson(const char *jsonString)
{
    StaticJsonDocument<2048> doc;
    DeserializationError error = deserializeJson(doc, jsonString);

    if (error)
    {
        LOG(LOG_ERROR, "JSON Fehlercode: %s", error.c_str());
        return false;
    }

    // --- LÄDT KONFIGURATION (Bleibt gleich) ---
    wifiSsid = doc[F("ssid")].as<String>();
    wifiPassword = doc[F("password")] | "";
    apSsid = doc[F("ap_ssid")] | "esp-aktor";
    apPassword = doc[F("ap_password")] | "MeinSicheresPasswort";

    int levelInt = doc[F("log_level")] | LOG_DEBUG;
    activeLogLevel = (LogLevel)levelInt;

    rpmConfig.pulses_per_revolution = doc[F("rpm_pulses")] | 1;
    timingConfig.trigger_offset_deg = doc[F("timing_offset")] | 60;
    coilConfig.primary_resistance_ohm = doc[F("primary_resistance_ohm")] | 0.2f;
    coilConfig.external_resistance_ohm = doc[F("external_resistance_ohm")] | 2.0f;

    // DEBUG-Ausgabe ...
    LOG(LOG_DEBUG, "--- GELADENE KONFIGURATION ---");
    LOG(LOG_DEBUG, "STA SSID: %s (Länge: %d)", wifiSsid.c_str(), wifiSsid.length());
    LOG(LOG_DEBUG, "STA PW Länge: %d", wifiPassword.length());
    LOG(LOG_DEBUG, "AP SSID: %s | AP PW Länge: %d", apSsid.c_str(), apPassword.length());
    LOG(LOG_DEBUG, "Log Level: %d", (int)activeLogLevel);
    LOG(LOG_DEBUG, "RPM Pulses: %d | Offset: %d deg", rpmConfig.pulses_per_revolution, timingConfig.trigger_offset_deg);

    // ----------------------------------------------------------------
    // NEU: MAP LADEN UND VALIDIERUNG (2D-Achsen-Format)
    // ----------------------------------------------------------------

    JsonObject map2d = doc[F("map2d")];
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
    return true;
}

bool loadConfig()
{
    File configFile = LittleFS.open(F("/config.json"), "r");
    if (configFile)
    {
        size_t size = configFile.size();
        std::unique_ptr<char[]> buf(new char[size + 1]);
        configFile.readBytes(buf.get(), size);
        buf.get()[size] = '\0';
        configFile.close();
        if (loadDataFromJson(buf.get()))
        {
            LOG(LOG_INFO, "Konfiguration aus Datei geladen.");
            return true;
        }
        else
        {
            LOG(LOG_WARN, "Laden aus Datei fehlgeschlagen. Datei ist möglicherweise korrupt.");
        }
    }
    if (loadDataFromJson(FALLBACK_CURVE_JSON))
    {
        LOG(LOG_WARN, "Fallback-Kennfeld/Konfiguration geladen.");
        return true;
    }
    return false;
}

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
    doc[F("timing_offset")] = timingConfig.trigger_offset_deg;
    doc[F("primary_resistance_ohm")] = coilConfig.primary_resistance_ohm;
    doc[F("external_resistance_ohm")] = coilConfig.external_resistance_ohm;

    // NEU: MAP IM 2D-ACHSEN-FORMAT SPEICHERN
    JsonObject map2d = doc.createNestedObject(F("map2d"));

    // 1. Achsen
    JsonArray rpmAxisArray = map2d.createNestedArray(F("rpm_axis"));
    for (int r : ignitionMap2D.rpm_axis)
    {
        rpmAxisArray.add(r);
    }

    JsonArray tpsAxisArray = map2d.createNestedArray(F("tps_axis"));
    for (int t : ignitionMap2D.tps_axis)
    {
        tpsAxisArray.add(t);
    }

    // 2. Datenmatrix
    JsonArray angleDataArray = map2d.createNestedArray(F("angle_data"));
    for (const auto &row : ignitionMap2D.angle_data)
    {
        JsonArray rowArray = angleDataArray.createNestedArray();
        for (int angle : row)
        {
            rowArray.add(angle);
        }
    }

    File configFile = LittleFS.open(F("/config.json"), "w");
    if (!configFile)
        return false;
    bool success = (serializeJson(doc, configFile) > 0);
    configFile.close();
    if (success)
        LOG(LOG_INFO, "Config gespeichert.");
    return true;
}

// =========================================================
// 7. SETUP und LOOP
// =========================================================

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
        loadDataFromJson(FALLBACK_CURVE_JSON);

    LOG(LOG_INFO, "WLAN Setup: Verbinde mit '%s' (PW-Länge: %d)", wifiSsid.c_str(), wifiPassword.length());

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

    if (WiFi.status() == WL_CONNECTED)
    {
        LOG(LOG_INFO, "✅ Verbunden! IP: %s", WiFi.localIP().toString().c_str());
        // Setup der Webserver-Routen (aus WebServer.cpp)
        setup_server_routes();
        server.begin();
        // Setup OTA (aus WebServer.cpp)
        setup_ota();
        advanceTicker.attach_ms(50, updateIgnitionParameters);
        debugTicker.attach(1, printLoopStatus);
        LOG(LOG_INFO, "System bereit. Heap: %d Bytes", ESP.getFreeHeap());
    }
    else
    {
        LOG(LOG_WARN, "Verbindung fehlgeschlagen. Starte AP-Modus.");
        startApMode = true;
        // Setup der Webserver-Routen (aus WebServer.cpp)
        setup_server_routes();
    }
}

void loop()
{
    ESP.wdtFeed();

    unsigned long start_loop_micros = micros();
    unsigned long now = millis();

    // 1. AP/STA Modus Handling
    if (startApMode)
    {
        if (WiFi.getMode() != WIFI_AP)
        {
            WiFi.mode(WIFI_AP);
            WiFi.softAP(apSsid.c_str(), apPassword.c_str());
            LOG(LOG_INFO, "AP IP: %s", WiFi.softAPIP().toString().c_str());
        }
        server.handleClient();
    }
    else
    {
        if (now - lastWebHandle >= WEB_HANDLE_INTERVAL)
        {
            server.handleClient();
            lastWebHandle = now;
        }
        if (now - lastOtaHandle >= OTA_HANDLE_INTERVAL)
        {
            // ArduinoOTA.handle() muss hier aufgerufen werden, da es globale Funktionalität ist.
            ArduinoOTA.handle();
            lastOtaHandle = now;
        }
    }

    // 2. Heap Check und Statistik-Aktualisierung
    if (now - lastHeapCheck >= HEAP_CHECK_INTERVAL)
    {
        checkHeapHealth();
        lastHeapCheck = now;
    }

    // Loop-Dauer messen und Statistik aktualisieren
    unsigned long current_duration = micros() - start_loop_micros;
    loop_durations_buffer[stats_index] = current_duration;
    stats_index = (stats_index + 1) % STATS_WINDOW_SIZE;

    if (++calc_counter >= CALC_FREQUENCY)
    {
        calculateLoopStatistics();
        calc_counter = 0;
    }
}