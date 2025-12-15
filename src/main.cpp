// main.cpp - Minimal Orchestrator Rumpf (Aktion 133)

// =========================================================
// 1. INCLUDES (System und Module)
// =========================================================
#include <Arduino.h>
#include <ESP.h>
#include "AppLogger.h"
#include "CoreDefs.h"
#include "SystemController.h" 
#include "SystemStats.h"
#include "IgnitionManager.h" // Für Struct-Definitionen (IgnitionState, RpmConfig)

extern "C"
{
#include "user_interface.h"
}

#pragma GCC optimize("O3")

// =========================================================
// 2. IRAM EXTERNS (NEU)
// =========================================================

// Wir greifen direkt auf die Variablen zu, die jetzt in IgnitionManager.cpp definiert sind.
// Diese MÜSSEN volatile sein, da sie in Interrupts und im Loop verwendet werden.
extern volatile IgnitionState ignition;
extern volatile SpeedState speedState;
extern RpmConfig rpmConfig; 
extern const int IGNITION_OUTPUT_PIN; // Aus IgnitionManager.h

// =========================================================
// 3. IRAM INTERRUPT HANDLER (Angepasst für volatile/direkten Zugriff)
// =========================================================

void IRAM_ATTR fireIgnitionOutput()
{
    // Die Logik bleibt gleich, da sie die extern deklarierten Pins nutzt.
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
        
        // Hier direkt auf die globale (extern volatile) rpmConfig zugreifen
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


// =========================================================
// 4. SETUP und LOOP (Reine Koordination)
// =========================================================

void setup()
{
    Serial.begin(115200);
    delay(100);

    setupModules(); // Delegiert alle Initialisierungen an den Controller
    
    LOG(LOG_INFO, "System-Setup abgeschlossen. Starte Haupt-Loop.");
}

void loop()
{
    unsigned long start_loop_micros = micros();
    ESP.wdtFeed();
    
    handleNetworkLoop(); 
    handleSystemMaintenance();

    unsigned long current_duration = micros() - start_loop_micros;
    recordLoopDuration(current_duration); 
}