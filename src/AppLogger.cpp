// src/AppLogger.cpp

#include "AppLogger.h"
#include "SystemConfig.h" // Für activeLogLevel
#include <Arduino.h>
#include <stdarg.h>
#include <string.h>
#include <vector> // NEU: Für std::vector

// =========================================================
// 1. GLOBALE VARIABLEN DEFINITIONEN
// =========================================================

// Definition des Puffers als Vektor, um push_back und size zu ermöglichen
std::vector<String> startupLogBuffer; 

// Definition der globalen Variable (Deklariert in main.cpp/SystemConfig.h)
extern bool serverStarted;

// Maximale Anzahl der Log-Einträge, die vor dem Start des Servers gespeichert werden
const int MAX_STARTUP_LOGS = 20;

// =========================================================
// 2. FUNKTIONEN
// =========================================================

void app_log(LogLevel level, const char *file, int line, int line_end, const char *format, ...)
{
    // Stellt sicher, dass das Level mindestens so hoch ist wie das aktive Level
    if (level > activeLogLevel)
    {
        return;
    }

    // Zeitstempel und Log-Level-Präfix
    char prefix[20];
    if (level == LOG_ERROR) {
        snprintf(prefix, sizeof(prefix), "[%lu] ERROR: ", millis());
    } else if (level == LOG_WARN) {
        snprintf(prefix, sizeof(prefix), "[%lu] WARN: ", millis());
    } else if (level == LOG_INFO) {
        snprintf(prefix, sizeof(prefix), "[%lu] INFO: ", millis());
    } else {
        snprintf(prefix, sizeof(prefix), "[%lu] DEBUG: ", millis());
    }

    char logBuffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(logBuffer, sizeof(logBuffer) - 1, format, args);
    va_end(args);

    String output = String(prefix) + logBuffer;

    // Speichern im Startup-Puffer, bevor der Server läuft
    // Korrektur: size() und push_back() auf Vektor-Objekt
    if (!serverStarted && startupLogBuffer.size() < MAX_STARTUP_LOGS)
    {
        startupLogBuffer.push_back(output);
    }
    
    // Ausgabe auf Serial (wird immer gemacht)
    Serial.println(output);
}