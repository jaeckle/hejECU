#ifndef APPLOGGER_H
#define APPLOGGER_H

#include "CoreDefs.h"
#include <vector>

// Log-Makro für automatische Datei- und Zeilennummer
#define LOG(level, format, ...) app_log(level, __FILE__, __LINE__, __LINE__, format, ##__VA_ARGS__)

// Funktion für das eigentliche Logging
void app_log(LogLevel level, const char* file, int line, int function_ptr, const char* format, ...);

// Externe Deklarationen (Definitionen in main.cpp!)
extern volatile LogLevel activeLogLevel;
extern std::vector<String> startupLogBuffer;
extern bool serverStarted; // Benötigt für Puffer-Logik

#endif // APPLOGGER_H