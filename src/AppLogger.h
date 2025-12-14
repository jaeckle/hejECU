// AppLogger.h

#ifndef APPLOGGER_H
#define APPLOGGER_H

#include "CoreDefs.h" 
#include <stdarg.h>
#include <vector>

// Log-Makro, das Dateiname und Zeilennummer automatisch übergibt
#define LOG(level, format, ...) app_log(level, __FILE__, __LINE__, __LINE__, format, ##__VA_ARGS__) 

// Extern Deklarierte Logger-Funktionen
extern void app_log(LogLevel level, const char* file, int line, int logLine, const char* format, ...);
extern void logCriticalEvent(unsigned long maxDuration, unsigned long timestamp, const char* status);

// Externe Variablen
extern std::vector<String> startupLogBuffer;
extern const size_t MAX_STARTUP_LOGS; 
extern const char *LOG_FILENAME;

#endif // APPLOGGER_H