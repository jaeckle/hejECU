#pragma once
#include <Arduino.h>
#include <memory>
#include <vector>

class I2C_Expander;

// =========================================================
// 1. ZUSTANDSSTRUKTUREN
// =========================================================

// Definition des Enums (kann in CoreDefs bleiben, wenn dort schon vorhanden)
enum SensorID {
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

// Extern Deklaration des Sensorzustands (Definition erfolgt in SensorManager.cpp)
extern struct SensorState {
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
} sensor; // <-- Behält den Namen der globalen Variable

// =========================================================
// 2. SENSOR-KLASSEN (Abstraktion der Hardware)
// =========================================================

class BaseSensor {
public:
    virtual ~BaseSensor() {}
    virtual float readValue() = 0;
    virtual unsigned long getMinInterval() = 0;
};

// Die SensorManager-Klasse (oder Funktions-Modul) organisiert die Tasks
struct SensorTask {
    std::unique_ptr<BaseSensor> sensor_ptr;
    unsigned long last_update_ms = 0;
    SensorID target_id;
};

extern std::vector<SensorTask> allSensorTasks;
extern std::vector<std::unique_ptr<I2C_Expander>> allExpanderTasks; // Wenn I2C_Expander ausgelagert wird

// =========================================================
// 3. FUNKTIONSDEKLARATIONEN
// =========================================================

void initializeSensors(); // Neue Funktion, um alle Sensor-Tasks zu initialisieren
void updateSensorState(SensorID id, float value);
void updateSensorsRoundRobin();
void updateSlowIOControllers();