// AppLogger.cpp

#include "AppLogger.h"
#include <Arduino.h>
#include <LittleFS.h>
#include <memory> 
#include <ESP.h>

// Externe Deklarationen, um activeLogLevel und serverStarted zu verwenden
extern volatile LogLevel activeLogLevel;
extern bool serverStarted; 

// --- WICHTIGE GLOBALE STATUSVARIABLEN (Definitionen) ---
const size_t MAX_STARTUP_LOGS = 50; 
std::vector<String> startupLogBuffer;
const char *LOG_FILENAME = "/critical_log.txt";

// ZENTRALER LOGGER DEFINITION 
void app_log(LogLevel level, const char* file, int line, int logLine, const char* format, ...) {
    if (level < activeLogLevel) return; 

    const char* levelStr = "UNKNOWN";
    switch (level) {
        case LOG_DEBUG: levelStr = "DEBUG"; break;
        case LOG_INFO:  levelStr = "INFO";  break;
        case LOG_WARN:  levelStr = "WARNUNG"; break;
        case LOG_ERROR: levelStr = "FEHLER"; break;
    }
    
    unsigned long timeMs = millis();
    char logBuffer[160]; 
    
    int len = snprintf(logBuffer, sizeof(logBuffer), "[%lu ms] [%s] (%s:%d) ", 
                       timeMs, levelStr, file, logLine);

    va_list args;
    va_start(args, format);
    vsnprintf(logBuffer + len, sizeof(logBuffer) - len, format, args);
    va_end(args);
    
    Serial.println(logBuffer);
    
    if (!serverStarted && startupLogBuffer.size() < MAX_STARTUP_LOGS) {
        startupLogBuffer.push_back(String(logBuffer)); 
    }
}

// IMPLEMENTIERUNG: Loggt kritische Ereignisse ins LittleFS
void logCriticalEvent(unsigned long maxDuration, unsigned long timestamp, const char* status) {
    if (!LittleFS.begin()) {
        LOG(LOG_ERROR, "LittleFS ist nicht bereit, kritisches Log kann nicht geschrieben werden.");
        return;
    }
    
    File logFile = LittleFS.open(LOG_FILENAME, "a");
    if (!logFile) {
        LOG(LOG_ERROR, "Konnte kritisches Log-Datei nicht öffnen/erstellen.");
        return;
    }
    
    char buffer[128];
    snprintf(buffer, sizeof(buffer), "[%lu ms] [CRIT] %s - Max Loop: %lu us\n", 
             timestamp, status, maxDuration);
             
    logFile.print(buffer);
    logFile.close();
    
    app_log(LOG_WARN, __FILE__, __LINE__, __LINE__, "Kritisches Event protokolliert: %s", status);
}