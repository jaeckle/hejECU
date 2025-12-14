#include "AppLogger.h"
#include "CoreDefs.h"
#include <stdarg.h>

// Max Logs für den Buffer
#define MAX_STARTUP_LOGS 25 

// HINWEIS: Die Definitionen von activeLogLevel und startupLogBuffer
// wurden nach main.cpp verschoben, um doppelte Definitionen zu vermeiden.

void app_log(LogLevel level, const char* file, int line, int function_ptr, const char* format, ...) {
    if (level < activeLogLevel) {
        return;
    }

    char logBuffer[256];
    const char* levelStr = "";

    switch (level) {
        case LOG_ERROR: levelStr = "[FEHLER]"; break;
        case LOG_WARN:  levelStr = "[WARNUNG]"; break;
        case LOG_INFO:  levelStr = "[INFO]"; break;
        case LOG_DEBUG: levelStr = "[DEBUG]"; break;
        // LOG_NONE wird ignoriert
        default: break; 
    }

    // 1. Erstelle den Zeitstempel und den Header (File/Line/Function)
    unsigned long timestamp = millis();
    snprintf(logBuffer, sizeof(logBuffer), "[%lu ms] %s (%s:%d) ", 
             timestamp, levelStr, file, line);

    // 2. Füge die variable Nachricht hinzu
    size_t len = strlen(logBuffer);
    va_list args;
    va_start(args, format);
    vsnprintf(logBuffer + len, sizeof(logBuffer) - len, format, args);
    va_end(args);

    // 3. Sende an die Serielle Schnittstelle
    Serial.println(logBuffer);

    // 4. In den Startup-Buffer schreiben, falls der Server noch nicht gestartet ist
    // startupLogBuffer ist jetzt vom Typ std::vector<String>
    if (!serverStarted && startupLogBuffer.size() < MAX_STARTUP_LOGS) {
        startupLogBuffer.push_back(String(logBuffer));
    }
}