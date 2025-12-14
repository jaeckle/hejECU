#include "AppLogger.h"
#include "CoreDefs.h"
#include <stdarg.h>
#include <vector>

#define MAX_STARTUP_LOGS 25 

// WICHTIG: Hier NICHT die globalen Variablen definieren! Nur extern benutzen.

void app_log(LogLevel level, const char* file, int line, int function_ptr, const char* format, ...) {
    if (level < activeLogLevel) {
        return;
    }

    // Stack-Fix: Puffer auf 512 Bytes erhöht
    char logBuffer[512]; 
    const char* levelStr = "";
    
    // ... (Logik zur Bestimmung von levelStr) ...

    // 1. Erstelle den Zeitstempel und den Header
    unsigned long timestamp = millis();
    // Verwenden Sie snprintf für den ersten Teil
    int header_len = snprintf(logBuffer, sizeof(logBuffer), "[%lu ms] %s (%s:%d) ", 
             timestamp, levelStr, file, line);

    // 2. Füge die variable Nachricht hinzu
    va_list args;
    va_start(args, format);
    // Hier verwenden wir den Header-Offset (header_len)
    vsnprintf(logBuffer + header_len, sizeof(logBuffer) - header_len, format, args);
    va_end(args);

    // 3. Sende an die Serielle Schnittstelle
    Serial.println(logBuffer);

    // 4. In den Startup-Buffer schreiben (falls nötig)
    if (!serverStarted && startupLogBuffer.size() < MAX_STARTUP_LOGS) {
        // HINWEIS: Dies erfordert, dass startupLogBuffer in main.cpp definiert ist
        // und in AppLogger.h extern deklariert ist.
        startupLogBuffer.push_back(String(logBuffer));
    }
}